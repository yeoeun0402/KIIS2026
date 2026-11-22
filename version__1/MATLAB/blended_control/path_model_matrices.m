function [A, B] = path_model_matrices(T)
% 정해진 경로 위 이산 상태공간 모델의 행렬 (x = [s; v; delta], u = [a; omega])
%   x(k+1) = A x(k) + B u(k)
%   전진: s(k+1) = s(k) + T v(k) + T^2/2 a(k),  v(k+1) = v(k) + T a(k)   (더블 인테그레이터, 한 주기 동안 a 일정)
%   조향: delta(k+1) = delta(k) + T omega(k)                                (인테그레이터)
A = [1 T 0;
     0 1 0;
     0 0 1];
B = [T^2 / 2  0;
     T        0;
     0        T];
end
