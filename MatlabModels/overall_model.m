%     Copyright 2017-2020 Dino Spiller (dinospiller@gmail.com)
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.

%     Overall model: the "M","C","K" matrixes and the coordinate-change and
%     decoupling matrix
m11 = m_P*r^2/4+r^2*Jd/(D^2)+m_w*r^2+J_w+J_m/rho^2;
m12 = m_P*r^2/4-r^2*Jd/(D^2)+m_w*r^2+J_w;
m13 = m_P*L*r/2-J_m/rho^2;
m33 = m_P*L^2+2*J_m/rho^2+JO;
c11 = psi/rho^2;
k33 = -m_P*g*L;
%syms m11 m12 m13 m33 c11 k33

M = [m11 m12 m13;...
     m12 m11 m13;...
     m13 m13 m33];
C = [c11    0       -c11;...
     0      c11     -c11;...
     -c11   -c11    2*c11];
K = [0    0    0;...
     0    0    0;...
     0    0    k33];
 A = [ zeros(3) eye(3);...
      -inv(M)*K -inv(M)*C];
 U =[1/rho*eye(2);-1/rho -1/rho];
 B = [ zeros(3);inv(M)]*U;
% B_d = B*D;

% coordinates change, in order to decouple the system. 
%  xb = r*(alpha + beta)/2
%  delta = r*(alpha-beta)/D
%  xb      r/2  r/2  0     alpha
%  delta = r/D  -r/D 0     beta
%  theta_P  0    0   1     theta_P
%syms r D

%matrix of base-change: S
S = [ r/2  r/2  0;...
      r/D  -r/D 0;...
      0     0   1];

% other coordinates change, to find the motor angle
%  alpha = theta_P+rho*alpha_mot
%  beta = theta_P+rho*beta_mot
%  theta_P = theta_P
Smot=[ rho  0   1;...
       0   rho  1;...
       0    0   1];

% now rewrite the whole system in the coordinates [xb delta theta_P]'
M_s = M*inv(S);
C_s = C*inv(S);
K_s = K*inv(S);

A_s = [ zeros(3) eye(3);...
     -inv(M_s)*K_s -inv(M_s)*C_s];
B_s = [ zeros(3);inv(M_s)]*U;

%decoupling matrix, built assuming:
%   tau_theta = 1  1   tau_L
%   tau_delta   1 -1   tau_R
Dec = inv([1 1;1 -1]);

%decoupled B_s matrix
B_sd = B_s*Dec;

%re-arrange the whole system in a more-convenient view
% the system's state vector were: [xb delta theta_P dot_xb dot_delta dot_theta_P]'
% now they becomes:[xb theta_P dot_xb dot_theta_P delta dot_delta]'
N=[ 1 0 0 0 0 0;...
    0 0 1 0 0 0;...
    0 0 0 1 0 0;...
    0 0 0 0 0 1;...
    0 1 0 0 0 0;...
    0 0 0 0 1 0];
A_sN=N*A_s*inv(N);
B_sdN=N*B_sd;
