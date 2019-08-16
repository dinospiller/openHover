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

%     Steering subsysyem model and controller

%% steering subsystem
states_steering = {'delta' 'dot_delta'};
A_t=A_sN(5:6,5:6);
B_t=B_sdN(5:6,2:2);
C_t=[1 1];
D_t=0;

sys_steer = ss(A_t,B_t,C_t,D_t,'statename',states_steering,'inputname','tau_delta','outputname','dot_delta');
sys_steer_d = c2d(sys_steer,Ts);
[A_td,B_td,C_td,D_td] = ssdata(sys_steer_d);
    
poles_steering=eig(A_t);
[tf_t_num tf_t_den] =ss2tf(A_t,B_t,C_t,D_t);

% design LQR for steering subsystem
%            delta;dot_delta
%Q_steer=diag([1;2]);
%Q_steer=diag([100;0.001]);% seems good for simulations with psi=1
Q_steer=diag([1;1]);
R_steer=0.1;

K_steer_c = lqr(A_t,B_t,Q_steer,R_steer)
K_steer = dlqr(A_td,B_td,Q_steer,R_steer)

% lqr-controlled system
A_tc = (A_t-B_t*K_steer_c);
B_tc = B_t;
C_tc = C_t;
D_tc = D_t;
sys_steer_c = ss(A_tc,B_tc,C_tc,D_tc);
% Now evaluate the stability of the controlled system
poles_steer_controlled = eig(A_tc);

figure()
subplot(2,1,1);
pzplot(sys_steer);
title('Poles and zeros of steering angle transfer function')
subplot(2,1,2);
pzplot(sys_steer_c);
title('Poles and zeros of the LQR controlled system')

%% design of PI controller for the steering subsystem
% zeros_steering = roots(tf_d_num(1,:));
tf_t=tf(tf_t_num,tf_t_den);

% reduce the degree of the polynomials
if((tf_t_num(end)==0)&& (tf_t_den(end)==0))
   tf_t= tf(tf_t_num(1:length(tf_t_num)-1),tf_t_den(1:length(tf_t_den)-1));
end


KP_t=2% nearly unity gain, since the process is almost sufficiently fast
% find the pole frequency of the open-loop system and set the zero of the
% PI at a frequency of 1/10 of that. 
omega_open_loop=tf_t_den(2);
% since in a PI, the pulsation of the zero is omega_zero=(Ki/Kp), and it
% has to be omega_zero=1/10*omega_open_loop, then:
%KI_t=omega_open_loop/10*KP_t;
KI_t=omega_open_loop/3*KP_t
% then the controller becomes:
Wpi_t=KI_t/s+KP_t;
% tf_pi_t=tf(Wpi_t)
% discretization:
Wpi_td = c2d(Wpi_t, Ts);
[numPI, denPI] = tfdata(Wpi_td, 'v');

%closed-loop controller and system: verification
Wloop_t=(tf_t*Wpi_t);
Wcl_t=(tf_t*Wpi_t)/(1+(tf_t*Wpi_t));

% figure()
% subplot(3,1,1);
% bode(tf_t);
% title('Frequency plot of transfer function of steering subsystem')
% subplot(3,1,2);
% margin(Wloop_t);
% title('Frequency plot of loop-gain in the PI-controlled steering subsysyem')
% subplot(3,1,3);
% margin(Wcl_t);
% title('PI-controlled steering subsysyem frequency response')


references_dot_delta=[  0 ,0;...
                        20, 0;...
                        23, 1;...
                        26, -1;...
                        29, 0;...
                        final_time, 0];
% references_dot_delta=[  0 ,0;...
%                         final_time, 0];