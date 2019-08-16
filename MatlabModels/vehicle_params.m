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

%     Vehicle parameters definition

%syms m_P r Jd D m_w J_w J_m rho L psi g JO n_poles n_hall r_gearbox tacho_to_angle

% % BLDC motor parameters (gearmotor)
PN = 500;                       % [W] Nominal power of each motor
iNom = 20;                        % [A] Approx nominal current .... See datasheet 
n_poles = 30;                   % [/] number of magnetical poles on the motor
n_winds = 27;                   % [/] number of windings on the motor
r_gearbox = 1;                  % [/] Gearbox ratio
rho = 1/r_gearbox;
UN = 36;                        % [V] Nominal motor voltage
ke = 0.694;                     % [V*s/rad] this is the motor voltage constant
Rs = 0.128;                     % [Ohm] I don't know.... it has to be measured (i.e. with the 4 wires method)
Ls = 166.8e-6;                  % [H] motor ph-ph inductance
kt = ke;                        % [Nm/A] torque constant: number of Nm for every ampere of motor
n_hall = 3;                     % number of hall sensors
% %omega_speed_ratio = 2*pi*r/(60*(n_poles/2)*r_gearbox);

% BLDC motor parameters (for RC model motor)
% PN = 2400;                      % [W] Nominal power of each motor
% iNom = 90;                        % [A] Approx nominal current .... See datasheet 
% n_poles = 14;                   % [/] number of magnetical poles on the motor
% n_winds = 12;                   % [/] number of windings on the motor
% r_gearbox = 115/12;             % [/] Gearbox ratio
% rho = 1/r_gearbox;
% UN = 29.6;                      % [V] Nominal motor voltage
% ke = 1/(270*2*pi/60);           % [V*s/rad] this is the motor voltage constant
% Rs = 15e-3;                     % [Ohm] I don't know.... it has to be measured (i.e. with the 4 wires method)
% Ls = 35e-6;                     % [H] motor ph-ph inductance
% kt = ke;                        % [Nm/A] torque constant: number of Nm for every ampere of motor
% n_hall = 3;                     % number of hall sensors
%omega_speed_ratio = 2*pi*r/(60*(n_poles/2)*r_gearbox);

iMAX = 2*iNom; %it is assumed that the pulse current is approx. double the nominal current

% accelerometer params
n_bit_accelero=16;              % number of bits of accelerometer

m_P=30;                           %[kg] mass of the person
L=0.1;                            %[m] half the height of the person
m_b=5;                            %[kg]mass of the base
b_h=0.05;                         %[m]base heigth
b_l=0.2;                          %[m]base length
J_b=1/12*m_b*(b_h^2+b_l^2);       %[kg*m^2]momentum of inertia of the base
r=0.1;                            %[m] radius of the wheel 
Jd=1/2*m_P*(0.25)^2;                 %[kg*m^2]momentum of inertia of the person, rotating around its vertical axis
D=0.5;%0.4;                       %[m] distance between wheels
m_w=2.5;                          %[kg] mass of a wheel
J_w=1/2*m_P*r^2;                  %[kg*m^2]momentum of inertia of the wheel
J_m=1/2*2.5*0.1^2;               %[kg*m^2]momentum of inertia of the motor
psi=0.1;%5.7e-4;%0.1 optimum                      %[Nm/(rad/s)] viscous friction coefficient of the motor(derived with experimet)
g=9.81;                           %[m/s^2] gravity
JO=1/3*m_P*(2*L)^2;               %[kg*m^2]momentum of inertia of the person, rotating around wheel's axis
c_s=50;%350;                          %[Nm/(rad/s)] viscous friction coefficient of physiological ankle model
k_s=850;%1440;                         %[Nm/rad] spring coefficient of physiological ankle model
tacho_to_angle = (2*pi)/(n_poles*n_hall);


omega_pend = sqrt(g/L);
f_pend = omega_pend/(2*pi);

