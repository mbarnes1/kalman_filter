function [ mu_x_t ] = kalman_filter( mu_x_t1, yt, u_t1 )
%KALMAN_FILTER Calculates mean state using Kalman filter
%   Inputs:
%       mu_x_t1: The previous mean state
%       yt: The observed noisy state
%       u_t1: The previous control input
%
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

mu_x(:,t) = kalman_filter(mu_x(:,t-1), y(t), u(t-1))
end