function setup_paths()
% MATLAB 폴더와 제어기 폴더(Pure-Pursuit, PID, test1, blended_control)를 경로에 추가. MATLAB을 켤 때마다 한 번 실행
%   >> cd MATLAB
%   >> setup_paths
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here, 'Pure-Pursuit'), fullfile(here, 'PID'), fullfile(here, 'test1'), ...
    fullfile(here, 'blended_control'));
end
