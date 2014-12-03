function [ mu_t, Sigma_t ] = kalman_filter( mu_t1, Sigma_t1, y, u_t1, A, B, C)
%KALMAN_FILTER Calculates mean state using Kalman filter
%   Inputs:
%       Past state
%           mu_t1: The previous mean state
%           Sigma_t1: The previous covariance of the belief
%       yt: The observed noisy state
%       u_t1: The previous control input
%       A: Linearized model parameters
%       B: Linearized model parameters
%       C: Linearized model parameters
%       sigmaX: State transition noise
%       sigmaY: Motion model noise
%
%   Outputs:
%       New state
%           mu_x_t: The new mean state
%           Sigma_x_t: The new covariance of the belief

R = .1*eye(length(A));  % State transition noise, no covariance terms. sigmaX
Q = 1.0*eye(length(C));  % Measurement model noise, no covariance terms. sigmaY

%% Incorporate control input
mu_bar = A*mu_t1 + B*u_t1;
Sigma_bar = A*Sigma_t1*A' + R;

%% Incorporate measurement
K = Sigma_bar*C'/(C*Sigma_bar*C' + Q);  % Kalman gain
mu_t = mu_bar + K*(y-C*mu_bar);
Sigma_t = (eye(length(C)) - K*C)*Sigma_bar;
end