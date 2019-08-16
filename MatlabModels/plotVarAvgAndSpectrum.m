function out=plotVarAvgAndSpectrum(variable,name,timevector,sampling_time)
    figure();
    n_samples=length(variable);
    %time plot
    avg_var=mean(variable);
    variance_var=var(variable);
    std_dev_var=sqrt(variance_var);
    subplot(2,1,1);
    plot(timevector,...
        variable,...
        timevector,...
        avg_var*ones(1,n_samples),...
        timevector,...
        avg_var+std_dev_var*ones(1,n_samples),'--r',...
        timevector,...
        avg_var-std_dev_var*ones(1,n_samples),'--r');
    xlabel('t (s)');
    ylabel('Amplitude');
    string_title='Time plot of signal "'+name+'", its average ='+string(avg_var)+' and standard deviation'+char(10)+' The variance is: '+ variance_var;
    title(string_title);
    % FFT plot
    Fs=1/sampling_time;
    sig_FFT = fft(variable-avg_var);% subtract average, to delete continous-contribute
    P2 = 20*log10(abs(sig_FFT/n_samples));
    P1 = P2(1:n_samples/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(n_samples/2))/n_samples;
    subplot(2,1,2);
    plot(f,P1) 
    string_title='Single-Sided Amplitude Spectrum of "'+name+'"';
    title(string_title)
    xlabel('f (Hz)')
    ylabel('|frequency envelope|')
end

