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

%     TDigital filters: used in the Simulink simulation and in the C code


%% DISCRETE_TIME FILTERS

%% Define the DT high-pass filters (for velocity estimation)
%   continuous-time filter definition 
% omega_HPF = 20;    % [rad/s] the pole is placed at that frequency
% omega_HFpole = 1000;    % [rad/s] another high-frequency pole
% numFc = s;     
% denFc = (1+s/omega_HPF)*(1+s/(omega_HFpole));       
% sysFc = numFc/denFc;%tf(numFc, denFc);       % s/(s/omega_HPF+1)
f_HPF = 5;% 2.5; 
omega_HPF = 2*pi*f_HPF;    % [rad/s] the pole is placed at that frequency
psi_HPF=0.707;
numFc = s;     
denFc = (s^2/omega_HPF^2+2*psi_HPF*s/omega_HPF+1);       
sysFc = numFc/denFc;%tf(numFc, denFc);       % s/(s/omega_HPF+1)

%   filter discretization
sysHPF = c2d(sysFc, Ts);
[numF, denF] = tfdata(sysHPF, 'v');

%% discrete-time integrator
sysInt=tf([0 1],[1 0]); % = 1/s
sysIntD = c2d(sysInt, Ts);
[numInt, denInt] = tfdata(sysIntD, 'v');

%% PLL (alternative way for derivation): try to design it considering "omega
% and psi"

% f_PLL = 5;
% omega_PLL = 2*pi*f_PLL;
% k_PLL=10;                 %OPTIMUM VALUES
% psi_PLL=0.707;
% tau_PLL=omega_PLL

f_PLL = 5;
omega_PLL = 2*pi*f_PLL;
k_PLL=15;
psi_PLL=0.707;
tau_PLL=omega_PLL;

%% accelero and gyro internal bandwidths (internal first-order LPF)

f_IMU_BW = 500; %Hz
omega_IMU_BW =2*pi*f_IMU_BW;
lpfIMUinternal = c2d(tf(1/(s/omega_IMU_BW + 1)), Ts);
[numIMUint, denIMUint] = tfdata(lpfIMUinternal, 'v');


% IMU offsets
imu_accx_offset = -0.33175;
imu_accz_offset = 0.3771;
imu_gyro_offset = -0.0140;

%% Complementary filter
alpha_c = 0.999;
beta_c = 1-alpha_c;

%% Kalman Filter (for IMU)
Q_kalman_IMU=[0.00000001, 0;0, 0.0001];
R_kalman_IMU=20;



%% current loop PI filter
Kp_i_loop=0.158;
Ki_i_loop=124;
mot_model=(1/(Rs+s*Ls));
curr_PI=(Kp_i_loop+Ki_i_loop/s);
iloop_PI =tf(curr_PI*mot_model/(1+curr_PI*mot_model));
iloop_PI_d=c2d(iloop_PI,Ts);
[numIloop, denIloop] = tfdata(iloop_PI_d, 'v');


%% Notch filter
f_notch = 300; %[Hz]
omega_notch = 2*pi*f_notch;
% the notch is the combination of two damped poles and two underdamped
% zeroes: define how many dB to subtract
dB_subtract=40; %the total dB you want to subtract
dB_poles=-5;
damp_poles=10^(dB_poles/20);
damp_zeroes= 10^((-dB_subtract+dB_poles)/20);
num_notch=1+2*s*damp_zeroes/omega_notch+s^2/omega_notch^2;
den_notch=1+2*s*damp_poles/omega_notch+s^2/omega_notch^2;
tf_notch=num_notch/den_notch;
filt_notch_d=c2d(tf_notch,Ts);
%figure();bode(filt_notch_d);

%%   second-order low-pass filter
f_LPF=50;%7; %[Hz]
omega_LPF=2*pi*f_LPF;
psi_LPF=1;
LPF_c=tf(1/(s^2/omega_LPF^2+2*psi_LPF*s/omega_LPF+1));
LPF_d = c2d(LPF_c, Ts);
%figure();bode(LPF_d)


%% IMU and torque filters: associate the corresponding filter, 
% thus decide if notch or LPF
filt_IMU_postproc=filt_notch_d;
filt_torques=LPF_d;

[numTorques, denTorques] = tfdata(filt_torques, 'v');
[numIMU, denIMU] = tfdata(filt_IMU_postproc, 'v');

%% disturbance torque, used to compensate backlash
dist_torque_ampl = 0;