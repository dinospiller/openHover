% accelero X
accx_ind=find(var_names=="accX");
plotVarAvgAndSpectrum(statevars(accx_ind,:),var_names(accx_ind),timevector,Ts);
% accelero Z
accz_ind=find(var_names=="accZ");
plotVarAvgAndSpectrum(statevars(accz_ind,:),var_names(accz_ind),timevector,Ts);
% gyro Y
gyro_ind=find(var_names=="gyroTheta");
plotVarAvgAndSpectrum(statevars(gyro_ind,:),var_names(gyro_ind),timevector,Ts);
% accelero_angle_LPF
%plotVarAvgAndSpectrum(statevars(4,:),var_names(4),timevector,Ts);

% % integrate the angular velocity, to obtain the angle
% % discrete-time integrator
% sysInt=tf([0 1],[1 0]); % = 1/s
% 
% sysIntD = c2d(sysInt, Ts);
% 
% gyro_out = statevars(gyro_ind,:);
% [statevars(gyro_ind,:), t , x]=lsim(sysIntD,statevars(gyro_ind,:));
% 
figure()
plot(timevector,statevars(:,:));
legend(var_names);
% 
% innovation_ind=find(var_names=="innovation");
% periodogram(statevars(innovation_ind,:),1/Ts,5,'Angle Fusion with Kalman')

%% plot FFT of accelerometer
% Fs=1/Ts;
% acc_FFT = fft(statevars(1,:));
% P2 = abs(acc_FFT/n_samples);
% P1 = P2(1:n_samples/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:(n_samples/2))/n_samples;
% figure();
% plot(f,P1) 
% title('Single-Sided Amplitude Spectrum of accelerometer out(t)')
% xlabel('f (Hz)')
% ylabel('|accelero noise(f)|')
% 
% %% plot FFT of gyro
% gyro_FFT = fft(gyro_out);
% P2 = abs(gyro_FFT/n_samples);
% P1 = P2(1:n_samples/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% f = Fs*(0:(n_samples/2))/n_samples;
% figure();
% plot(f,P1) 
% title('Single-Sided Amplitude Spectrum of gyroscope out(t)')
% xlabel('f (Hz)')
% ylabel('|gyro noise(f)|')