function [y, hist] = lpf_window(hist, u, ell, wcur)
% 논문 Eq.8–9의 Low-pass Filter 한 스텝 (Chen et al., ITSC 2018, PP_PI)
%   y = Σ_{j=1..ℓ-1} δ_previous,j · w_previous  +  δ_current · w_current        (Eq.8)
%   w_previous = (1 - w_current) / (ℓ - 1)                                       (Eq.9)
%     δ_current  = u : 이번 스텝에 계산한 조향각 (K_pp δ_pp + δ_p + δ_i)
%     δ_previous     : 직전 ℓ-1개 스텝의 최종 조향각 출력 (필터를 거친 값)
%     ℓ = 필터 창 길이 (현재 1개 + 이전 ℓ-1개),  w_current = 이번 값의 가중치
%   가중치 합 = (ℓ-1) w_previous + w_current = 1 → 일정한 입력은 그대로 통과 (DC 이득 1)
%   ℓ = 1이면 y = u (필터 없음). ℓ = 2이면 y = w u + (1 - w) y[k-1] (1차 저역통과와 같음)
%   hist : 이전 출력 (최신이 1번), 길이 ≥ ℓ-1.  반환 hist는 y를 맨 앞에 넣은 것
ell = round(ell);
if ell <= 1
    y = u;
else
    if numel(hist) < ell - 1
        error("lpf_window: 이전 출력이 %d개 필요한데 %d개뿐입니다.", ell - 1, numel(hist));
    end
    wprev = (1 - wcur) / (ell - 1);
    y = wcur * u + wprev * sum(hist(1:ell - 1));
end
hist = [y; hist(1:end - 1)];
end
