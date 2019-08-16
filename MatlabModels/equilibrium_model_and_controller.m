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

%     Equilibrium model and controller

%% Balancing subsystem ("equilibrium")
A_e=A_sN(1:4,1:4);
B_e=B_sdN(1:4,1:1);
% C_e=[   1 0 0 0  ;...
%         0 1 0 0  ;...
%         0 0 1 0  ;...
%         0 0 0 1 ];
% D_e=[   0  ;...
%         0  ;...
%         0  ;...
%         0  ];

% % test of controllability with the only "theta_P" variables
% C_e=[   0 1 0 0  ;...
%         0 0 0 1 ];
% D_e=[   0  ;...
%         0  ];
C_e=[   0 1 0 0  ;...
        0 0 1 0 ];
D_e=[   0  ;...
        0  ];
states = {'Xb' 'theta_p' 'vb' 'theta_dot_p'};
inputs = {'tau_theta' };
%outputs = {'Xb' 'theta_p' 'vb' 'theta_dot_p'};
%outputs = {'theta_p' 'theta_dot_p'}; % this row tells that even with the only "theta_p" variables, the system is controllable
outputs = {'theta_p' 'vb'};
sys_ssb = ss(A_e,B_e,C_e,D_e,'statename',states,'inputname',inputs,'outputname',outputs);

% Check poles of the state-space (evaluate stability of the system)
poles_equilibrium = eig(A_e);
[V,P]= eig(A_e);

% figure()
% pzplot(sys_ssb);
% title('Poles and zeros of the open-loop system')


CTB=ctrb(sys_ssb);

if(rank(CTB) == length(A_e))
    fprintf('Ok MAC.... The system is controllable.\n')
else
    fprintf('Gosh MAC!... The system is NOT controllable!\n')
end

% observability of the system
obsv_rank = rank(obsv(A_e,C_e));

%% LQR parameters for the continuos time model
% states: Xb,theta,dotXb,dotTheta
%Q=diag([5;15;2;0.1]); %used for simulations

%Q=diag([1e-10;3;0;0.1]); % all poles real with R=0.1

%Q=diag([10;5;10;30]); % all poles real with R=0.1 and more attention to angular velocity
%Q=diag([0.01;40;50;0]); % try to minimize self-oscillation
Q=diag([10;3;10;0.1]); % experiments Wood
%Q=diag([1e-10;5;0.1;20]); % segway GOOOD!
R=0.1;
%R=0.001;
K = lqr(A_e,B_e,Q,R);

% Controllable parameters 
Ac = (A_e-B_e*K);
Bc = B_e;
Cc = C_e;
Dc = D_e;
sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);


% figure()
% pzplot(sys_cl);
% title('Poles and zeros of the controlled system')

% %stability margins
% Gl= K*inv(s*eye(length(A_e))-A_e)*B_e;
% figure();
% margin(Gl)

%Step response- example
% figure
% t1 = 0:0.01:10;
% stimuli = [0.5*ones(1,size(t1,2))];
% %stimuli = [0.5*ones(1,ceil(size(t1,2)*0.1)) 0*ones(1,(size(t1,2)-(ceil(size(t1,2)*0.1))))];
% [y1,t1,x1] = lsim(sys_cl,stimuli,t1);
% plot(t1,y1,t1,stimuli/100);
% legend('Horizontal position[m]','Person angle[rads]','horizontal velocity[m/s]','Person angle ratio [rad/s]','stimuli [Nm]/100');
% grid on;
% title('Step transient -- LQR Control')

%%  Model discretization (for the equilibrium subsystem)
sysPd = c2d(sys_ssb, Ts);        %   discrete-time model (LTI obj)
[Ad,Bd,Cd,Dd] = ssdata(sysPd);  %   discrete-time model (state-space matrices) 

% DLQR parameters for the model: discrete-time variant of LQR
Kd = dlqr(Ad,Bd,Q,R)

% Controllable parameters 
Acd = (Ad-Bd*K);
Bcd = Bd;
Ccd = Cd;
Dcd = Dd;

% Step response- example
% figure
% t1 = 0:0.01:10;
% r = 0.5*ones(size(t1));
% [y1,t1,x1] = lsim(sysPd,r,t1);
% [AX,H1,H2] = plotyy(t1,y1(:,1),t1,y1(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','Body Angle[rads]')
% set(get(AX(2),'Ylabel'),'String','Wheel speed[rad]')
% grid on;
% title('Step transient -- LQR Control')

poles_rd = eig(Acd);
sysCld=ss(Acd,Bcd,Ccd,Dcd,Ts);

% plot of the poles of the controlled system
% figure()
% pzplot(sysCld);
% title('Poles and zeros of the discrete-time controlled system')

% reference vector, for the linear velocity 
%                (time,value)

references_dot_Xb=[ 0, 0;...
                    2.5, 0.5;...                  
                    5, 0;...
                    final_time, 0];
% references_dot_Xb=[ 0, 0;...
%                     final_time, 0];