%% experiments... various!
clear all;
close all;
% s=tf('s')
% omega_n =4.08;
% xi=0.496
% num = 1
% den = (s^2/omega_n^2)+2*xi*s/omega_n+1
% tfunc = tf(num/den)
% bode(tfunc)

%% HPF experiment on manual manipulation
Ts=1/100;
omega_HPF = 20;    % [rad/s] the pole is placed at that frequency
numFc = [1 0];     
denFc = [1/omega_HPF  1];       
sysFc = tf(numFc, denFc);       % s/(s/omega_HPF+1)

%   filter discretization
sysHPF = c2d(sysFc, Ts)
[numF, denF] = tfdata(sysHPF, 'v');
figure
subplot(1,2,1);
bode(sysHPF);
final_time = 3;
t=0:Ts:final_time;
x=sin(2*pi*1*t);
dot_x=lsim(sysHPF,x,t); % simulated with matlab
% my own simulation algorithm
dot_x_my(1)=0;
for ind=2:size(t)
    dot_x_my(ind)=denF(2)*dot_x_my(ind-1)+numF(1)*x(ind)+numF(2)*x(ind-1);
end
    
subplot(1,2,2);
plot(t,x, t, dot_x, t, dot_x_my)
legend('x','dotx','dotx my')