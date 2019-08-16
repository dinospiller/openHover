% Filename: f_updating.m
% Author: Dino Spiller
% Date: 22/06/2014
%
% Description: function for calculating the filtered values of the state,
% used in the Kalman filter. The system is modeled as:
% x(t+1)= F*x(t)+B*u(t)+w(t)
% y(t)= C*x(t)+D*u(t)+v(t)
% and var{[w(t),v(t)]'} = [Q 0]' [0 R]'

% Arguments:
% P: previous-step variance of estimated state
% x: previous-step estimated state
% C: C matrix
% R: R matrix
% y: current y(t) sample
% F: F matrix
% D: D matrix
% u: current u(t) sample

% Returns:
% x: filtered estimation of the state
% P: variance of the error of estimated state
function [P,x]=f_updating(P,x,C,R,y,F,D,u)

L=P*C'*inv(C*P*C'+R);
e=y-C*x-D*u;
I=eye(size(L*C));

P=(I-L*C)*P*(I-L*C)'+L*R*L';
x=x+L*e;