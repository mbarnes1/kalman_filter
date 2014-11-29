function [ mu_t, Sigma_t ] = kalman_filter( mu_t1, Sigma_t1, yt, u_t1 )
%KALMAN_FILTER Calculates mean state using Kalman filter
%   Inputs:
%       Past state
%           mu_x_t1: The previous mean state
%           Sigma_x_t1: The previous covariance of the belief
%       yt: The observed noisy state
%       u_t1: The previous control input
%
%   Outputs:
%       New state
%           mu_x_t: The new mean state
%           Sigma_x_t: The new covariance of the belief
% Steps:
%   Incorporate control u_t1 to calculate belief bar (shifts, but does not incorporate measurement)
%       - needs past mean (mu_x_t1) to calculate new mean
%       - needs past covariance (Sigma) and Rt (covariance for Gaussian
%       noise in state transition)
%   Calculate Kalman gain
%       - needs Qt (covariance of noise in measurement model)
%   Update belief
%
%   Need three different covariance matrices
%       Motion model noise from sigmaX
%       State observation noise from sigmaY
%       Sigma part of my belief, this is a new variable. What is initial
%       value?
R = sigmaX*eye(length(A));  % State transition noise, no covariance terms
Q = sigmaY*eye(length(C));  % Measurement model noise, no covariance terms

%% Incorporate control input
mu_bar = A*mu_x_t1 + B*u_t1;
Sigma_bar = A*Sigma_t1*A' + R;

%% Incorporate measurement
K = Sigma_bar*C'*inverse(C*Sigma_bar*C' + Q);  % Kalman gain
mu_t = mu_bar + K*(y-C*mu_bar);
Sigma_t = (eye(length(C)) - K*C)*Sigma_bar;
end