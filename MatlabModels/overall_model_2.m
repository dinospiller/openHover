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

%syms m11_2 m12_2 m13_2 m14_2 m33_2 m44_2 c11_2 c33_2 c34_2 k33_2 k44_2
m11_2 = m_P*r^2/4+r^2*Jd/(D^2)+m_w*r^2+J_w+m_b*r+J_m/rho^2;
m12_2 = m_P*r^2/4-r^2*Jd/(D^2)+m_w*r^2+J_w+m_b*r;
m13_2 = -J_m/rho^2;
m14_2 = m_P*L*r/2;
m33_2 = 2*J_m/rho^2+J_b;
m44_2 = m_P*L^2+JO;
c11_2 = psi/rho^2;
c33_2 = 2*psi/rho^2+c_s;
c34_2 = -c_s;
k33_2 = k_s;
k44_2 = -m_P*g*L+k_s;


M_2 = [m11_2 m12_2 m13_2 m14_2;...
     m12_2 m11_2 m13_2 m14_2;...
     m13_2 m13_2 m33_2 0  ;...
     m14_2 m14_2 0   m44_2];
C_2 = [c11_2    0       -c11_2    0;...
     0      c11_2     -c11_2    0;...
     -c11_2   -c11_2    c33_2     c34_2;...
     0      0       c34_2     -c34_2];
K_2 = [0    0    0    0;...
     0    0    0    0;...
     0    0    k33_2 -k33_2;...
     0    0    -k33_2 k44_2];
 U_2=[1/rho 0 0;0 1/rho 0; -1/rho -1/rho -1; 0 0 1];
 A_2 = [ zeros(4) eye(4);...
      -inv(M_2)*K_2 -inv(M_2)*C_2];
 B_2 = [ zeros(4);inv(M_2)]*U_2;
% B_d = B*D;

% coordinates change, in order to decouple the system. 
%  xb = r*(alpha + beta)/2
%  delta = r*(alpha-beta)/D
%  xb      r/2   r/2   0   0     alpha
%  delta = r/D  -r/D   0   0     beta
%  theta_b    0    0   1   0     theta_b
%  theta_P    0    0   0   1     theta_P
%syms r D

%matrix of base-change: S
S_2 = [ r/2  r/2  0  0;...
      r/D  -r/D 0  0;...
      0     0   1  0;...
      0     0   0  1];

% now rewrite the whole system in the coordinates [xb delta theta_P]'
M2_s = M_2*inv(S_2);
C2_s = C_2*inv(S_2);
K2_s = K_2*inv(S_2);

A2_s = [ zeros(4) eye(4);...
     -inv(M2_s)*K2_s -inv(M2_s)*C2_s];
B2_s = [ zeros(4);inv(M2_s)]*U_2;

%decoupling matrix, built assuming:
%   tau_theta = 1  1  0 tau_L
%   tau_s       0  0  1 tau_R
%   tau_delta   1 -1  0 tau_s
Dec_2 = inv([1 1 0;0 0 1; 1 -1 0]);

%decoupled B_s matrix
B2_sd = B2_s*Dec_2;

%re-arrange the whole system in a more-convenient view
% the system's state vector was: [xb delta theta_b theta_P dot_xb dot_delta dot_theta_b dot_theta_P]'
% now it becomes: [xb dot_xb theta_b dot_theta_b theta_P dot_theta_P delta dot_delta]'
N_2=[ 1 0 0 0 0 0 0 0;...
    0 0 0 0 1 0 0 0;...
    0 0 1 0 0 0 0 0;...
    0 0 0 0 0 0 1 0;...
    0 0 0 1 0 0 0 0;...
    0 0 0 0 0 0 0 1;...
    0 1 0 0 0 0 0 0;...
    0 0 0 0 0 1 0 0];
A2_sN=N_2*A2_s*inv(N_2);
B2_sdN=N_2*B2_sd;

