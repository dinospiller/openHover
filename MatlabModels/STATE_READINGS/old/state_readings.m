close all;
clear all;

% % %try to convert a 4-byte int8 array, to float
% % % the number is -12.7562, coded in IEEE 754: 0xc14c1965
% % hex_number = [ '65' ;'19'; '4c'; 'c1']
% % dec_number = hex2dec(hex_number)
% % float_num = typecast(uint8(dec_number), 'single')
% 
n_samples = 1000;
ser = serial('/dev/tty.SELF_BALANCE_115200-SPP','BaudRate',115200);
fopen(ser);
if(ser.BytesAvailable)
    flushinput(ser);
end

var_names=["Xb";"thetaP";"dotXb";"dotThetaP";"delta";"dotDelta";"torqueEq";"torqueSteer";"tachoL";"tachoR"];
num_vars=length(var_names);

timevector=zeros(1,n_samples);
statevars=zeros(num_vars,n_samples);
payload=zeros(1,1);
payload_len=0;
payload_read_ind=1;
crc=uint16(0);
WAIT_HEADER=1;
WAIT_LEN=2;
READ_PAYLOAD=3;
READ_CRC1=4;
READ_CRC2=5;
READ_TAIL=6;
curr_reading_state=WAIT_HEADER;

%% test_vector
testvector=[char(02);...
            char(03);...
            char(67);...
            char(105);...
            char(0);...
            char(245);...
            char(79);...
            char(03)];
testvector_ind=1;

%% Reading from serial
% header = 0x02
% tail = 0x03
% format:
% header  len   payload CRC16 tail
%  (1B)   (1B)   (n*B)  (2B)  (1B)
ind=1;
while ind<=n_samples
    if(ser.BytesAvailable)
        curr_char=uint8(fread(ser,1));
%         curr_char=uint8(testvector(testvector_ind));
%         testvector_ind = testvector_ind+1; 
%         if(testvector_ind>length(testvector))
%             testvector_ind =1;
%         end
        switch curr_reading_state
            case WAIT_HEADER
                if curr_char==2
                    curr_reading_state = curr_reading_state+1;
                end
            case WAIT_LEN
                payload_len=curr_char;
                curr_reading_state = curr_reading_state+1;
            case READ_PAYLOAD
                if payload_read_ind < payload_len
                    payload(payload_read_ind)=curr_char;
                    payload_read_ind = payload_read_ind+1;
                else % if arrived to last payload char
                    payload(payload_read_ind)=curr_char;
                    payload_read_ind=1;
                    curr_reading_state = curr_reading_state+1;
                end
            case READ_CRC1
                crc = uint16(curr_char);
                crc = bitshift(crc,8);
                curr_reading_state = curr_reading_state+1;
            case READ_CRC2
                crc = crc + uint16(curr_char);
                curr_reading_state = curr_reading_state+1;
            case READ_TAIL
                [crcbit crchex crcdec]=crc16(payload);
                if(crc==crcdec)
                    %statevars = decode_payload(payload);
                    if(length(payload)==(num_vars*4)+4)
                        timevector(ind)=typecast(flip(uint8(payload(1:4))),'int32');
                        disp([ind timevector(ind)]);
                        for varind=1:(num_vars)
                            str_ind=((varind)*4)+1;
                            dec_number = flip(uint8(payload(str_ind:str_ind+3)));
                            if((varind==find(var_names=="tachoL"))|(varind==find(var_names=="tachoR")))
                                statevars(varind,ind) = typecast(dec_number, 'int32');
                            else
                                statevars(varind,ind) = typecast(dec_number, 'single');
                            end
                        end
                        ind=ind+1;
                    else
                        disp('-------->>>PAYLOAD_LENGTH ERROR----->>>');
                    end
                else
                    disp("-------->>>CRC ERROR!!!!------>>");
                end
                payload_len=0;
                crc=0;
                curr_reading_state = WAIT_HEADER;
        end
    end
end;
fclose(ser);
plot(timevector,statevars);
%plot(timevector,statevars(1,:),'r',timevector,statevars(2,:),'g',timevector,statevars(3,:), '--r' ,timevector,statevars(4,:),'--g',timevector,statevars(5,:), 'b' ,timevector,statevars(6,:),'--b' ,timevector,statevars(7,:),'k',timevector,statevars(8,:), 'm' );
legend(var_names);

