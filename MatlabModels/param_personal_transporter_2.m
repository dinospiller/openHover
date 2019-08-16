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

%     Two-wheels, inverse pendulum transporter model and controller
clc;
close all;
clear all;


% simulation parameters
s=tf('s');
f_PWM = 168e6/35000;            % [Hz] PWM frequency (see ST code)
Ts =  0.01;                     % [s] sampling time (loop executes at this frequency)
final_time=30;                  % [s] end time
n_samples= final_time/Ts;
timevector = linspace(0,final_time,n_samples)';

run('vehicle_params.m');
run('overall_model.m');
run('overall_model_2.m');
run('equilibrium_model_and_controller.m');
run('steering_model_and_controller.m');
run('digital_filters');
run('header_file_control_params.m');  % write params into C header file

%  run('bluetooth_param_writer');  % write params via bluetooth radio
% 
run('personal_transporter_model2_with_nonidealities.slx');
% disp('ATTENTION: it is compulsory to run the simulation, in order to compare true and simulated data!')
% disp('After simulation complete, come back here and press ENTER')
% pause
% 
% run('state_readings_experim.m'); % start simulation and collect data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %controllability of the equilibrium system:
% A_e=A_sN(1:6,1:6);
% B_e=B_sdN(1:6,1:2);
% A_d=A_sN(7:8,7:8);
% B_d=B_sdN(6:8,3:3);
% 
% % C_e=[   1 0 0 0 0 0 ;...
% %         0 1 0 0 0 0 ;...
% %         0 0 1 0 0 0 ;...
% %         0 0 0 1 0 0 ];
% % D_e=[   0 0 ;...
% %         0 0 ;...
% %         0 0 ;...
% %         0 0 ];
% C_e=eye(6);
% D_e=zeros(6,2);
% 
% 
% states = {'Xb' 'vb' 'theta_b' 'dot_theta_b' 'theta_P' 'dot_theta_P'};
% inputs = {'tau_theta' 'tau_s'};
% outputs = {'Xb' 'vb' 'theta_b' 'dot_theta_b' 'theta_P' 'dot_theta_P'};
% sys_ss = ss(A_e,B_e,C_e,D_e,'statename',states,'inputname',inputs,'outputname',outputs);
% 
% [tf_b_num,tf_b_den] = ss2tf(A_e,B_e,C_e,D_e,1);
% 
% poles_equilibrium = eig(A_e) 
% 
% % Check poles of the t.f. (evaluate stability of the system)
% poles_steering=eig(A_d);
% 
% CTB=ctrb(sys_ss);
% if(rank(CTB) == length(A_e))
%     fprintf('Ok MAC.... The system is controllable.\n')
% else
%     fprintf('Gosh MAC!... The system is NOT controllable: controllability matrix has rank = %d \n',rank(CTB))
% end
% 
% % observability of the system
% obsv_rank = rank(obsv(A_e,C_e))
% %% LQR parameters for the continuos time model
% %Q=diag([1;100;1;0.001;0.001;0.001]);
% Q=diag([1;1;1;10;100;1]);
% %Q=diag([1;0.1;.01;0.1;100;1]);
% %xxQ=diag([100;100;0.1;0.001;0.001;0.001]);
% %R=diag([0.001 0.1]);
% R=diag([0.1 100]);
% Lqr_K = lqr(A_e,B_e,Q,R);
% 
% % Controllable parameters 
% Ac = (A_e-B_e*Lqr_K);
% Bc = B_e;
% Cc = C_e;
% Dc = D_e;
% sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% % Now evaluate the stability of the controlled system
% poles_controlled = eig(Ac);
% 
% %Step response- example
% figure
% t1 = 0:0.01:10;
% %stimuli = [1;0]*0.5*ones(size(t1));
% stimuli = [100;0]*[0.5*ones(1,ceil(size(t1,2)*0.1)) 0*ones(1,(size(t1,2)-(ceil(size(t1,2)*0.1))))];
% %stimuli = 0.5*ones(size(t1));
% [y1,t1,x1] = lsim(sys_cl,stimuli,t1);
% plot(t1,x1,t1,stimuli/100);
% legend('Horizontal position[m]','Base angle[rads]','Person angle[rads]','horizontal velocity[m/s]','Base angle ratio [rad/s]','Person angle ratio [rad/s]','stimuli [Nm]/100');
% grid on;
% title('Step transient -- LQR Control')

