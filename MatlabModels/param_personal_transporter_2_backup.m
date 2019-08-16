
clc;
clear all;
close all;
% simulation parameters
s=tf('s');
f_PWM = 168e6/35000;            % [Hz] PWM frequency (see ST code)
t_simul = 15;
f_IMU = 250;                    % [Hz] IMU frequency in STM discovery board
Tc = 1e-3;
n_samples = t_simul/Tc;
t = 0:Tc:t_simul;
Tc_IMU = 1/f_IMU;
rTc = Tc_IMU/Tc;


% BLDC motor parameters
PN = 2400;                      % [W] Nominal power of each motor
iN = 90;                        % [A] Approx nominal current .... See datasheet -- Non ho idea quanti A tiri quel motorino :D  
n_poles = 14;                   % [/] number of magnetical poles on the motor
n_winds = 12;                   % [/] number of windings on the motor
r_gearbox = 115/16;             % [/] Gearbox ratio
rho = 1/r_gearbox;
UN = 29.6;                      % [V] Nominal motor voltage
ke = 1/(270*2*pi/60);           % [V*s/rad] this is the motor voltage constant
Rs = 15e-3;                     % [Ohm] I don't know.... it has to be measured (i.e. with the 4 wires method)
Ls = 35e-6;                     % [H] motor ph-ph inductance
kt = ke;                        % [Nm/A] torque constant: number of Nm for every ampere of motor
%omega_speed_ratio = 2*pi*r/(60*(n_poles/2)*r_gearbox);

% %% symbolic representation of the system's matrices
% % the following matrices are those defined in the problem
%syms m_P m_b J_b r Jd D m_w J_w J_m rho L psi g JO c_s k_s

m_P=80;             %[kg] mass of the person
m_b=5;              %[kg]mass of the base
b_h=0.05;           %[m]base heigth
b_l=0.2;            %[m]base length
J_b=1/12*m_b*(b_h^2+b_l^2); %[kg*m^2]momentum of inertia of the base
r=0.1;              %[m] radius of the wheel 
Jd=1/2*m_P*0.2^2;   %[kg*m^2]momentum of inertia of the person, rotating around its vertical axis
D=0.4;              %[m] distance between wheels
m_w=1;              %[kg] mass of a wheel
J_w=1/2*m_P*r^2;    %[kg*m^2]momentum of inertia of the wheel
J_m=1/2*0.5*0.03^2; %[kg*m^2]momentum of inertia of the motor
rho=1/6;            %[ ] reduction ratio of the motor to wheel
L=0.9;              %[m] half the height of the person
psi=5.7e-4;         %[Nm/(rad/s)] viscous friction coefficient of the motor(derived with experimet)
g=9.81;             %[m/s^2] gravity
JO=1/3*m_P*(2*L)^2; %[kg*m^2]momentum of inertia of the person, rotating around wheel's axis
c_s=350;            %[Nm/(rad/s)] viscous friction coefficient of physiological ankle model
k_s=1440;           %[Nm/rad] spring coefficient of physiological ankle model

%omega_n= sqrt(k_s/JO);
omega_n= sqrt(k_s/J_b)
xi = c_s/(2*sqrt(k_s*JO));
omega_d = omega_n*sqrt(1-xi^2);
omega_pend = sqrt(g/L);
f_pend = omega_pend/(2*pi)

%syms m11 m12 m13 m14 m33 m44 c11 c33 c34 k33 k44
m11 = m_P*r^2/4+r^2*Jd/(D^2)+m_w*r^2+J_w+m_b*r+J_m/rho^2;
m12 = m_P*r^2/4-r^2*Jd/(D^2)+m_w*r^2+J_w+m_b*r;
m13 = -J_m/rho^2;
m14 = m_P*L*r/2;
m33 = 2*J_m/rho^2+J_b;
m44 = m_P*L^2+JO;
c11 = psi/rho^2;
c33 = 2*psi/rho^2+c_s;
c34 = -c_s;
k33 = k_s;
k44 = -m_P*g*L+k_s;


M = [m11 m12 m13 m14;...
     m12 m11 m13 m14;...
     m13 m13 m33 0  ;...
     m14 m14 0   m44];
C = [c11    0       -c11    0;...
     0      c11     -c11    0;...
     -c11   -c11    c33     c34;...
     0      0       c34     -c34];
K = [0    0    0    0;...
     0    0    0    0;...
     0    0    k33 -k33;...
     0    0    -k33 k44];
 U=[1/rho 0 0;0 1/rho 0; -1/rho -1/rho -1; 0 0 1];
 A = [ zeros(4) eye(4);...
      -inv(M)*K -inv(M)*C];
 B = [ zeros(4);inv(M)]*U;
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
S = [ r/2  r/2  0  0;...
      r/D  -r/D 0  0;...
      0     0   1  0;...
      0     0   0  1];

% now rewrite the whole system in the coordinates [xb delta theta_P]'
M_s = M*inv(S);
C_s = C*inv(S);
K_s = K*inv(S);

A_s = [ zeros(4) eye(4);...
     -inv(M_s)*K_s -inv(M_s)*C_s];
B_s = [ zeros(4);inv(M_s)]*U;

%decoupling matrix, built assuming:
%   tau_theta = 1  1  0 tau_L
%   tau_s       0  0  1 tau_R
%   tau_delta   1 -1  0 tau_s
Dec = inv([1 1 0;0 0 1; 1 -1 0]);

%decoupled B_s matrix
B_sd = B_s*Dec;

%re-arrange the whole system in a more-convenient view
% the system's state vector was: [xb delta theta_b theta_P dot_xb dot_delta dot_theta_b dot_theta_P]'
% now it becomes: [xb dot_xb theta_b dot_theta_b theta_P dot_theta_P delta dot_delta]'
N=[ 1 0 0 0 0 0 0 0;...
    0 0 0 0 1 0 0 0;...
    0 0 1 0 0 0 0 0;...
    0 0 0 0 0 0 1 0;...
    0 0 0 1 0 0 0 0;...
    0 0 0 0 0 0 0 1;...
    0 1 0 0 0 0 0 0;...
    0 0 0 0 0 1 0 0];
A_sN=N*A_s*inv(N);
B_sdN=N*B_sd;

%controllability of the equilibrium system:
A_e=A_sN(1:6,1:6);
B_e=B_sdN(1:6,1:2);
%B_e=B_sdN(1:6,1:1);
A_d=A_sN(7:8,7:8);
B_d=B_sdN(6:8,3:3);

% C_e=[   1 0 0 0 0 0 ;...
%         0 1 0 0 0 0 ;...
%         0 0 1 0 0 0 ;...
%         0 0 0 1 0 0 ];
% D_e=[   0 0 ;...
%         0 0 ;...
%         0 0 ;...
%         0 0 ];
C_e=eye(6);
D_e=zeros(6,2);


states = {'Xb' 'vb' 'theta_b' 'dot_theta_b' 'theta_P' 'dot_theta_P'};
inputs = {'tau_theta' 'tau_s'};
%inputs = {'tau_theta'};
%outputs = {'Xb' 'vb' 'theta_b' 'dot_theta_b'};
outputs = {'Xb' 'vb' 'theta_b' 'dot_theta_b' 'theta_P' 'dot_theta_P'};
sys_ss = ss(A_e,B_e,C_e,D_e,'statename',states,'inputname',inputs,'outputname',outputs);

[tf_b_num,tf_b_den] = ss2tf(A_e,B_e,C_e,D_e,1);

poles_equilibrium = eig(A_e) 

% torque to base angle: useful for transfer function stability analisys
numerator=tf_b_num(3,:);
base_angle_tf = tf(numerator,tf_b_den)
figure()
subplot(1,2,1);
pzplot(base_angle_tf);
title('Poles and zeros of base angle transfer function')
subplot(1,2,2);
bode(base_angle_tf);
zeros_base_angle = roots(numerator)

% torque to person angle
numerator=tf_b_num(5,:);
person_angle_tf = tf(numerator,tf_b_den)
figure()
subplot(1,2,1);
pzplot(person_angle_tf);
title('Poles and zeros of person angle transfer function')
subplot(1,2,2);
bode(person_angle_tf);
zeros_person_angle = roots(numerator)


% Check poles of the t.f. (evaluate stability of the system)


poles_steering=eig(A_d);

CTB=ctrb(sys_ss);
if(rank(CTB) == length(A_e))
    fprintf('Ok MAC.... The system is controllable.\n')
else
    fprintf('Gosh MAC!... The system is NOT controllable: controllability matrix has rank = %d \n',rank(CTB))
end

% observability of the system
obsv_rank = rank(obsv(A_e,C_e))
%% LQR parameters for the continuos time model
%Q=diag([1;100;1;0.001;0.001;0.001]);
Q=diag([1;1;1;10;100;1]);
%Q=diag([1;0.1;.01;0.1;100;1]);
%xxQ=diag([100;100;0.1;0.001;0.001;0.001]);
%R=diag([0.001 0.1]);
R=diag([0.1 100]);
Lqr_K = lqr(A_e,B_e,Q,R);

% Controllable parameters 
Ac = (A_e-B_e*Lqr_K);
Bc = B_e;
Cc = C_e;
Dc = D_e;
sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% Now evaluate the stability of the controlled system
poles_controlled = eig(Ac);

%Step response- example
figure
t1 = 0:0.01:10;
%stimuli = [1;0]*0.5*ones(size(t1));
stimuli = [100;0]*[0.5*ones(1,ceil(size(t1,2)*0.1)) 0*ones(1,(size(t1,2)-(ceil(size(t1,2)*0.1))))];
%stimuli = 0.5*ones(size(t1));
[y1,t1,x1] = lsim(sys_cl,stimuli,t1);
plot(t1,x1,t1,stimuli/100);
legend('Horizontal position[m]','Base angle[rads]','Person angle[rads]','horizontal velocity[m/s]','Base angle ratio [rad/s]','Person angle ratio [rad/s]','stimuli [Nm]/100');
grid on;
title('Step transient -- LQR Control')

% %%  Model discretization
% 
% sysPd = c2d(sys_cl, Tc_IMU);     %   discrete-time model (LTI obj)
% [Ad,Bd,Cd,Dd] = ssdata(sysPd);  %   discrete-time model (state-space matrices) 
% 
% % DLQR parameters for the model
% %Q = C'*C
% %r11=0.1;
% %Q=diag([0.1;100;1;10])
% %R = diag([r11 r11]);
% %R=0.001;
% Kd = dlqr(Ad,Bd,Q,R)
% 
% % Controllable parameters 
% Acd = (Ad-Bd*K);
% Bcd = Bd;
% Ccd = Cd;
% Dcd = Dd;
% 
% % Step response- example
% % figure
% % t1 = 0:0.01:10;
% % r = 0.5*ones(size(t1));
% % [y1,t1,x1] = lsim(sysPd,r,t1);
% % [AX,H1,H2] = plotyy(t1,y1(:,1),t1,y1(:,2),'plot');
% % set(get(AX(1),'Ylabel'),'String','Body Angle[rads]')
% % set(get(AX(2),'Ylabel'),'String','Wheel speed[rad]')
% % grid on;
% % title('Step transient -- LQR Control')
% 
% poles_rd = eig(Acd)

%[-m_P*L*u(8)^2*r*sin(u(4))/2+u(9)*u(8)*(u(5)-u(6)) ;  -m_P*L*u(8)^2*r*sin(u(4))/2+u(9)*u(8)*(u(6)-u(5))  ;  0   ;  -m_P*g*L*sin(u(4))-u(9)*(u(5)-u(6))^2 ]
K_4s=[-0.0032 -275.8365  -31.6946 -128.4467];