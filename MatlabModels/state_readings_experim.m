close all;

n_samples = final_time/Ts;


ser = serial('/dev/tty.SELF_BALANCE_115200-SPP','BaudRate',115200);
fopen(ser);
ser.Terminator=3;
for ind=1:100
    if(ser.BytesAvailable)
        fread(ser, ser.BytesAvailable);
    end
end
disp('---> OK: ready for receiving DATA! <---')

startSimulation(ser);

var_names=["Xb";"thetaP";"dotXb";"dotThetaP";"delta";"dotDelta";"torqueEq";"torqueSteer";"tachoL";"tachoR";"ref Xb";"ref dotXb";"ref dotDelta"];
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
buf_ind=1;

buffer='';
while ind<=n_samples
    %buffer=fread(ser,ser.BytesAvailable);
    buffer=fscanf(ser, '%c');
    while(buf_ind<=length(buffer))
        curr_char=uint8(buffer(buf_ind));
        
        if(buf_ind==length(buffer))
            buffer='';
            buf_ind=1;
        else
            buf_ind=buf_ind+1;
        end
        
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
                        %disp([ind timevector(ind)]);
                        if(mod(ind,100)==0)
                            disp(ind/100);
                        end
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
                        disp(payload);
                    end
                else
                    disp("-------->>>CRC ERROR!!!!------>>");
                    buffer=fscanf(ser, '%c');% in this case, read a message and try to re-sync
                end
                payload_len=0;
                crc=0;
                curr_reading_state = WAIT_HEADER;
        end
    end
end;
fclose(ser);

timevector = timevector-timevector(1);
timevector = timevector*Ts;

run('state_reading_postproc.m');

function out = startSimulation(ser)
    COMM_START_SIM_SEQUENCE = 44;
    msg = MessageComposer;
    msg = msg.setCommand(COMM_START_SIM_SEQUENCE,true);
    msg = msg.addCrcHeadersAndSend(ser);
end


