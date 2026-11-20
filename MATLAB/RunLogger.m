classdef RunLogger < handle
    % 실행 1회의 기록을 모아서 결과 폴더에 저장
    %   - 매 스텝 add(row)로 한 행씩 추가 (미리 할당한 배열)
    %   - autosaveSec 마다 run.mat 중간 저장 (MATLAB이 죽어도 남도록)
    %   - finish()에서 timeseries.csv, run.mat, reference_path.csv, summary.txt,
    %     trajectory.png/.fig, 사용한 코드 사본(src/) 저장. 여러 번 불러도 한 번만 저장
    %   - 주행 함수에서 onCleanup(@() logger.finish())로 걸어두면
    %     Q/Esc, 창 닫기, 오류, Ctrl+C 어느 경우든 그때까지의 기록이 저장됨

    properties
        folder                      % 결과 폴더
        cols                        % 열 이름 (단위 포함)
        data                        % 기록 배열
        n = 0                       % 기록된 행 수
        ref                         % 레퍼런스 경로
        params                      % 차량·제어기 파라미터
        meta                        % 실행 정보
        sourceFiles = strings(0, 1) % run 폴더에 복사할 코드 파일
        notes = strings(0, 1)       % 메모 (예: 주행 중 파라미터 변경)
        completed = false           % 목표 바퀴 수를 다 돌았는지
        reason = "중단"             % 끝난 이유
        autosaveSec = 10
        finished = false
        summary = struct()
    end
    properties (Access = private)
        lastSave
    end

    methods
        function obj = RunLogger(folder, cols, nMax, ref, params, meta)
            obj.folder = folder;
            obj.cols = cols;
            obj.data = nan(nMax, numel(cols));
            obj.ref = ref;
            obj.params = params;
            obj.meta = meta;
            obj.lastSave = tic;
            if ~isfolder(folder), mkdir(folder); end
        end

        function add(obj, row)
            if obj.n == size(obj.data, 1)
                obj.data = [obj.data; nan(size(obj.data))];   % 모자라면 두 배로
            end
            obj.n = obj.n + 1;
            obj.data(obj.n, :) = row;
            if toc(obj.lastSave) > obj.autosaveSec
                obj.saveMat();
                obj.lastSave = tic;
            end
        end

        function note(obj, msg)
            obj.notes(end + 1, 1) = string(msg);
        end

        function T = table(obj)
            T = array2table(obj.data(1:obj.n, :), 'VariableNames', obj.cols);
        end

        function saveMat(obj)
            log = obj.table();
            ref = obj.ref;
            params = obj.params;
            meta = obj.meta;
            meta.notes = obj.notes;
            meta.completed = obj.completed;
            meta.reason = obj.reason;
            meta.saved = string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
            meta.summary = obj.summary;
            save(fullfile(obj.folder, 'run.mat'), 'log', 'ref', 'params', 'meta');
        end

        function S = finish(obj)
            if obj.finished, S = obj.summary; return; end
            obj.finished = true;
            T = obj.table();
            obj.summary = obj.computeSummary(T);
            S = obj.summary;

            % 저장 하나가 실패해도 나머지는 저장되도록 각각 시도
            obj.tryRun(@() writetable(T, fullfile(obj.folder, 'timeseries.csv')), 'timeseries.csv');
            obj.tryRun(@() obj.saveMat(), 'run.mat');
            obj.tryRun(@() obj.writeRef(), 'reference_path.csv');
            obj.tryRun(@() obj.writeSummary(), 'summary.txt');
            obj.tryRun(@() obj.plotRun(T), 'trajectory.png');
            obj.tryRun(@() obj.copySources(), 'src/');
            fprintf('기록 저장: %s\n', obj.folder);
        end
    end

    methods (Access = private)
        function c = ctrlParams(obj)
            % 제어기 파라미터 (pure pursuit: params.pp, PID: params.pid)
            if isfield(obj.params, 'pp'), c = obj.params.pp; else, c = obj.params.pid; end
        end

        function S = computeSummary(obj, T)
            S.folder = string(obj.folder);
            S.mode = string(obj.ctrlParams().mode);
            S.completed = obj.completed;
            S.reason = string(obj.reason);
            S.steps = obj.n;
            S.e_lat_mean_abs = NaN; S.e_lat_rms = NaN; S.e_lat_mse = NaN;
            S.e_lat_max_abs = NaN; S.e_lat_std = NaN; S.e_head_rms_deg = NaN;
            S.e_head_max_deg = NaN; S.lap_time = NaN; S.distance = NaN;
            S.sat_ratio = NaN; S.comp_ms_mean = NaN; S.comp_ms_max = NaN;
            if obj.n == 0, return; end
            e = T.e_lat_m;
            S.e_lat_mean_abs = mean(abs(e));
            S.e_lat_mse      = mean(e.^2);
            S.e_lat_rms      = sqrt(S.e_lat_mse);
            S.e_lat_max_abs  = max(abs(e));
            S.e_lat_std      = std(e);
            S.e_head_rms_deg = rad2deg(sqrt(mean(T.e_head_rad.^2)));
            S.e_head_max_deg = rad2deg(max(abs(T.e_head_rad)));
            S.lap_time       = T.t_s(end);
            S.distance       = T.progress_m(end);
            S.sat_ratio      = mean(T.delta_sat);
            S.comp_ms_mean   = mean(T.comp_ms);
            S.comp_ms_max    = max(T.comp_ms);
        end

        function writeRef(obj)
            r = obj.ref;
            R = table((1:r.N)', r.x, r.y, r.s, r.psi, r.kappa, ...
                'VariableNames', {'idx', 'x_m', 'y_m', 's_m', 'psi_rad', 'kappa_1pm'});
            writetable(R, fullfile(obj.folder, 'reference_path.csv'));
        end

        function writeSummary(obj)
            S = obj.summary;
            pp = obj.ctrlParams();
            names = fieldnames(pp);
            vals = cellfun(@(f) valueText(pp.(f)), names, 'UniformOutput', false);
            yesNo = ["아니오", "예"];
            lines = [
                "실행: " + obj.meta.runName
                "경로: " + string(obj.ref.name) + sprintf(" (한 바퀴 %.2f m)", obj.ref.L)
                "모드: " + S.mode
                "파라미터: " + strjoin(string(names) + "=" + string(vals), ", ")
                "완주: " + yesNo(S.completed + 1) + " (" + S.reason + sprintf("), 진행 거리 %.2f m", S.distance)
                sprintf("주행 시간: %.2f s (%d 스텝)", S.lap_time, S.steps)
                sprintf("횡오차 e_lat [m]: 평균|e| %.4f, RMS %.4f, MSE %.6f, 최대|e| %.4f, 표준편차 %.4f", ...
                    S.e_lat_mean_abs, S.e_lat_rms, S.e_lat_mse, S.e_lat_max_abs, S.e_lat_std)
                sprintf("방향오차 e_head [deg]: RMS %.2f, 최대 %.2f", S.e_head_rms_deg, S.e_head_max_deg)
                sprintf("조향 한계 도달 비율: %.1f %%", 100 * S.sat_ratio)
                sprintf("제어 계산시간 [ms]: 평균 %.2f, 최대 %.2f", S.comp_ms_mean, S.comp_ms_max)
                ];
            if ~isempty(obj.notes)
                lines = [lines; "메모:"; "  " + obj.notes];
            end
            writelines(lines, fullfile(obj.folder, 'summary.txt'));
        end

        function plotRun(obj, T)
            p = obj.params.vehicle;
            fig = figure('Visible', 'off', 'Position', [100 100 1500 650]);
            ax1 = axes(fig, 'Position', [0.04 0.1 0.55 0.82]);
            draw_map(ax1, p);
            plot(ax1, obj.ref.x, obj.ref.y, 'y-', 'LineWidth', 1.5);
            if obj.n > 0
                plot(ax1, T.x_m, T.y_m, 'r-', 'LineWidth', 1.2);
                plot(ax1, T.x_m(1), T.y_m(1), 'ko', 'MarkerFaceColor', 'w');
            end
            legend(ax1, {'레퍼런스', '실제 궤적', '출발'}, 'Location', 'southoutside', 'Orientation', 'horizontal');
            title(ax1, strrep(obj.meta.runName, '_', '\_'));

            ax2 = axes(fig, 'Position', [0.66 0.58 0.32 0.34]);
            ax3 = axes(fig, 'Position', [0.66 0.1 0.32 0.34]);
            if obj.n > 0
                plot(ax2, T.t_s, T.e_lat_m, 'b-'); grid(ax2, 'on');
                plot(ax3, T.t_s, rad2deg(T.delta_rad), 'b-', T.t_s, rad2deg(T.delta_cmd_rad), 'r--');
                grid(ax3, 'on'); legend(ax3, {'\delta', '\delta_{cmd}'});
            end
            ylabel(ax2, 'e_{lat} [m]'); xlabel(ax2, 't [s]');
            title(ax2, sprintf('RMS %.3f m, 최대 %.3f m', obj.summary.e_lat_rms, obj.summary.e_lat_max_abs));
            ylabel(ax3, '조향각 [deg]'); xlabel(ax3, 't [s]');
            exportgraphics(fig, fullfile(obj.folder, 'trajectory.png'), 'Resolution', 110);
            exportgraphics(fig, fullfile(obj.folder, 'trajectory.svg'), 'ContentType', 'vector');   % 벡터 그림
            savefig(fig, fullfile(obj.folder, 'trajectory.fig'));
            close(fig);
        end

        function copySources(obj)
            if isempty(obj.sourceFiles), return; end
            d = fullfile(obj.folder, 'src');
            if ~isfolder(d), mkdir(d); end
            for f = obj.sourceFiles(:)'
                if isfile(f), copyfile(f, d); end
            end
        end

        function tryRun(~, fn, what)
            try
                fn();
            catch err
                warning("%s 저장 실패: %s", what, err.message);
            end
        end
    end
end

function s = valueText(v)
if isnumeric(v) || islogical(v)
    s = strjoin(string(v), " ");
else
    s = string(v);
end
end
