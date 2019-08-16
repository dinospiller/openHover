% Filename: f_prediction.m
% Author: Dino Spiller
% Date: 22/06/2014
%
% Description: function for calculating the prediction of the state,
% used in the Kalman filter. The system is modeled as:
% x(t+1)= F*x(t)+B*u(t)+w(t)
% y(t)= C*x(t)+D*u(t)+v(t)
% and var{[w(t),v(t)]'} = [Q 0]' [0 R]'

% Arguments:
% P: previous-step variance of predicted state
% x: previous-step predicted state
% F: F matrix
% R: R matrix
% S: S matrix
% y: current y(t) sample
% B: B matrix
% D: D matrix
% u: current u(t) sample

% Returns:
% x: prediction of the state
% P: variance of the error of predicted state
function[P,x]=f_prediction(P,x,F,R,S,y,Q,B,u)

x=F*x+S*inv(R)*y+B*u;
P=F*P*F'+Q;