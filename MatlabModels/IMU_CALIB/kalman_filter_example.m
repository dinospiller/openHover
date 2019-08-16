%  Alternative way, for calculation of parameters: Kalman filter, initilization

P = 0*eye(2);
Ts= 0.01;

F = [1 -Ts; 0 1];
G = [Ts ; 0];
D = 0;
% R = 10;
% Q = diag([0.000001,10000000]);
R = 2000;
Q = diag([0.00001,0.1]);
S = zeros(2,1);
C = [1 , 0];
e = zeros(1,n_samples);



% the output is the accelerometer output
y = statevars(1,:);
x = [0;0];



% der_c=tf([1 0],[1/(2*pi*1000000000) 1]); % = s/(1+s/wn)
% Ts=0.01;
% der_d = c2d(der_c, Ts);
% %[gyro_out, t , x]=lsim(der_d,statevars(2,:));
% [gyro_out, t , x]=lsim(der_c,statevars(2,:),timevector);
%gyro_out = statevars(2,:);


%recursive kalman filtering
for i=1:n_samples
 
    e(i) = (y(i)-C*x(:,i)); 
    
	[P,x(:,i)]=f_updating(P,x(:,i),C,R,y(i),F,D,gyro_out(i));
    if(i~=n_samples)
        [P,x(:,i+1)]=f_prediction(P,x(:,i),F,R,S,y(i),Q,G,gyro_out(i));
    end
       
end

figure()
plot(timevector,x(1,:),timevector,y,'--',timevector,e,timevector,statevars(3,:));
legend(["predicted angle";"accelero out";"error";"complementary filter"]);
title('Accelerometer and kalman predicted angle')

periodogram(e,1/Ts,5,'Angle Fusion with unfiltered accelero')


%% ALTERNATIVE: low-pass filter the accelero and retry kalman tuning

% experiment: try to low-pass accelero data:
s=tf('s');
f_LPF=5; %[Hz]
LPF_c=tf(1/((1+s/(2*pi*f_LPF))*(1+s/(2*pi*f_LPF))));
LPF_d = c2d(LPF_c, Ts);

[accelero_LPF, t , x]=lsim(LPF_d,statevars(1,:));
figure()
subplot(2,2,1)
plot(timevector,statevars(1,:), timevector,accelero_LPF);
legend(["accelero","accelero LPF"]);
subplot(2,2,3)
% execute a frequency analisys of accelerometer's output
acc_FFT = fft(statevars(1,:));
Fs=1/Ts;
P2 = abs(acc_FFT/n_samples);
P1 = P2(1:n_samples/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(n_samples/2))/n_samples;
plot(f,P1) 
title('Absolute Spectrum of accelerometer out')
xlabel('f (Hz)')
ylabel('|P1(f)|')
subplot(2,2,4)
% execute a frequency analisys of accelerometer's output
acc_filt_FFT = fft(accelero_LPF);
Fs=1/Ts;
P2 = abs(acc_filt_FFT/n_samples);
P1 = P2(1:n_samples/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(n_samples/2))/n_samples;
plot(f,P1) 
title('Absolute Spectrum of filtered accelerometer out ')
xlabel('f (Hz)')
ylabel('|P1(f)|')


Rf = 10;
Qf = diag([0.01,0.07]);

% the output is the FILTERED accelerometer output
y = accelero_LPF;
x = [0;0];

%recursive kalman filtering
for i=1:n_samples
 
    e(i) = (y(i)-C*x(:,i)); 
    
	[P,x(:,i)]=f_updating(P,x(:,i),C,Rf,y(i),F,D,gyro_out(i));
    if(i~=n_samples)
        [P,x(:,i+1)]=f_prediction(P,x(:,i),F,Rf,S,y(i),Qf,G,gyro_out(i));
    end
       
end

figure()
plot(timevector,x(1,:),timevector,y,'--',timevector,e,timevector,statevars(3,:));
legend(["predicted angle";"accelero out";"error";"complementary filter"]);
title('LP filtered accelerometer and kalman predicted angle')

periodogram(e,1/Ts,5,'Angle Fusion with Kalman LP filtered accelero')
