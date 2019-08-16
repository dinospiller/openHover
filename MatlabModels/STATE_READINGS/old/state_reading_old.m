close all;
clear all;

% % %try to convert a 4-byte int8 array, to float
% % % the number is -12.7562, coded in IEEE 754: 0xc14c1965
% % hex_number = [ '65' ;'19'; '4c'; 'c1']
% % dec_number = hex2dec(hex_number)
% % float_num = typecast(uint8(dec_number), 'single')
% 
n_samples = 1000;
ser = serial('/dev/cu.SELF_BALANCE_115200-SPP','BaudRate',115200);
fopen(ser);
if(ser.BytesAvailable)
    fread(ser, ser.BytesAvailable);
end

disp('---> OK: ready for receiving DATA! <---')

var_names=["Xb";"thetaP";"dotXb";"dotThetaP";"delta";"dotDelta";"torqueEq";"torqueSteer";"tachoL";"tachoR"];
num_vars=length(var_names);

timevector=zeros(1,n_samples);
statevars=zeros(num_vars,n_samples);

%% Reading from serial
% header = 0x02
% tail = 0x03
% format:
% header  len   payload CRC16 tail
%  (1B)   (1B)   (n*B)  (2B)  (1B)
ser.Terminator=3;
ind=0;
while ind<=n_samples
    A = fscanf(ser, '%c');
    if(ind==0)% skip first: migth be truncated
        ind=ind+1
    else 
        payload=A(3:length(A)-3);% waste header and tail
        if(length(payload)==(num_vars*4)+4);
            timevector(ind)=typecast(flip(uint8(payload(1:4))),'int32');
            for varind=1:(num_vars)
                str_ind=((varind)*4)+1;
                dec_number = flip(uint8(payload(str_ind:str_ind+3)));
                if((varind==find(var_names=="tachoL"))|(varind==find(var_names=="tachoR")))
                    statevars(varind,ind) = typecast(dec_number, 'int32');
                else
                    statevars(varind,ind) = typecast(dec_number, 'single');
                end
            end
            ind=ind+1
        else
            disp('erroneous payload:')
            disp(payload)
        end
    end
end;
fclose(ser);
%plot(timevector,statevars);
plot(timevector,statevars(1,:),'r',timevector,statevars(2,:),'g',timevector,statevars(3,:), '--r' ,timevector,statevars(4,:),'--g',timevector,statevars(5,:), 'b' ,timevector,statevars(6,:),'--b' ,timevector,statevars(7,:),'k',timevector,statevars(8,:), 'm' );
legend(var_names);

% t=linspace(1,n_samples,n_samples);
% 
% plot(t,angle,t,angle_filt);
% legend('angle','angle filt combined');