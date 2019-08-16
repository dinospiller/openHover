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
final_time=35;                  % [s] end time
n_samples= final_time/Ts;
timevector = linspace(0,final_time,n_samples)';

run('vehicle_params.m');
run('overall_model.m');
run('equilibrium_model_and_controller.m');
run('steering_model_and_controller.m');
run('digital_filters');
run('header_file_control_params.m');  % write params into C header file

%  run('bluetooth_param_writer');  % write params via bluetooth radio
% 
%  run('personal_transporter_model1_with_nonidealities.slx');
% % disp('ATTENTION: it is compulsory to run the simulation, in order to compare true and simulated data!')
% % disp('After simulation complete, come back here and press ENTER')
% pause
% 
% run('state_readings_experim.m'); % start simulation and collect data
