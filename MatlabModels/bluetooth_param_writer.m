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

COMM_GET_SELF_BALANCE_PAR = 38;


COMM_SET_REMOTE_JOYSTICK = 45;



ser = serial('/dev/tty.SELF_BALANCE_115200-SPP','BaudRate',115200);
fopen(ser);

setControllerParams(Ts*1000,...
                    n_poles,...
                    kt,...
                    R,...
                    rho,...
                    D,...
                    Kd,...
                    sysHPF,...
                    sysIntD,...
                    Wpi_td,...
                    filt_IMU_postproc,...
                    alpha_c,...
                    beta_c,...
                    K_steer,...
                    imu_accx_offset,...
                    imu_accz_offset,...
                    imu_gyro_offset,...
                    dist_torque_ampl,...
                    omega_notch,...
                    filt_torques,...
                    ser)

resetAllSetpointLinVelocity(ser);
for i = 1:length(references_dot_Xb)
    addSetpointLinVelocity(references_dot_Xb(i,1),references_dot_Xb(i,2),ser);
end
resetAllSetpointYawVelocity(ser);
for i = 1:length(references_dot_delta)
    addSetpointYawVelocity(references_dot_delta(i,1),references_dot_delta(i,2),ser);
end


fclose(ser);



function out = addSetpointLinVelocity(time,value,ser)
    COMM_ADD_SETPOINT_LIN_VELOCITY = 40;
    msg = MessageComposer;
    msg = msg.setCommand(COMM_ADD_SETPOINT_LIN_VELOCITY,true);
    msg = msg.addFloat(time);
    msg = msg.addFloat(value);
    msg = msg.addCrcHeadersAndSend(ser);
end

function out = resetAllSetpointLinVelocity(ser)
    COMM_RESET_SETPOINTS_LIN_VELOCITY = 41;
    msg = MessageComposer;
    msg = msg.setCommand(COMM_RESET_SETPOINTS_LIN_VELOCITY,true);
    msg = msg.addCrcHeadersAndSend(ser);
end

function out = addSetpointYawVelocity(time,value,ser)
    COMM_ADD_SETPOINT_YAW_VELOCITY = 42;
    msg = MessageComposer;
    msg = msg.setCommand(COMM_ADD_SETPOINT_YAW_VELOCITY,true);
    msg = msg.addFloat(time);
    msg = msg.addFloat(value);
    msg = msg.addCrcHeadersAndSend(ser);
end

function out = resetAllSetpointYawVelocity(ser)
    COMM_RESET_SETPOINTS_YAW_VELOCITY = 43;
    msg = MessageComposer;
    msg = msg.setCommand(COMM_RESET_SETPOINTS_YAW_VELOCITY,true);
    msg = msg.addCrcHeadersAndSend(ser);
end



function out = setControllerParams(loopPeriodMs,...
                                    n_poles,...
                                    kt,...
                                    r_wheel,...
                                    rho,...
                                    D_interwheel,...
                                    LQRGainsvect,...
                                    HPFvelTF,...
                                    integratorTF,...
                                    steeringTF,...
                                    filt_IMU_postproc,...
                                    alpha_c,...
                                    beta_c,...
                                    LQR_steer_vect,...
                                    accx_offset,...
                                    accz_offset,...
                                    gyro_offset,...
                                    dist_torque_ampl,...
                                    omega_notch,...
                                    filt_torques,...
                                    ser)
    COMM_SET_SELF_BALANCE_PAR = 39;
    msg = MessageComposer;
    msg = msg.setCommand(COMM_SET_SELF_BALANCE_PAR,true);
    msg = msg.addInt8(loopPeriodMs);
    msg = msg.addInt8(n_poles);
    msg = msg.addFloat(kt);
    msg = msg.addFloat(r_wheel);
    msg = msg.addFloat(rho);
    msg = msg.addFloat(D_interwheel);
    msg = msg.addFloat(LQRGainsvect(1));
    msg = msg.addFloat(LQRGainsvect(2));
    msg = msg.addFloat(LQRGainsvect(3));
    msg = msg.addFloat(LQRGainsvect(4));
    [numF, denF] = tfdata(HPFvelTF, 'v');
    msg = msg.addFloat(numF(1));
    msg = msg.addFloat(numF(2));
    msg = msg.addFloat(numF(3));
    msg = msg.addFloat(denF(2));
    msg = msg.addFloat(denF(3));
    [numF, denF] = tfdata(integratorTF, 'v');
    msg = msg.addFloat(numF(1));
    msg = msg.addFloat(numF(2));
    msg = msg.addFloat(denF(2));
    [numF, denF] = tfdata(steeringTF, 'v');
    msg = msg.addFloat(numF(1));
    msg = msg.addFloat(numF(2));
    msg = msg.addFloat(denF(2));
    [numF, denF] = tfdata(filt_IMU_postproc, 'v');
    msg = msg.addFloat(numF(1));
    msg = msg.addFloat(numF(2));
    msg = msg.addFloat(numF(3));
    msg = msg.addFloat(denF(2));
    msg = msg.addFloat(denF(3));
    msg = msg.addFloat(alpha_c);
    msg = msg.addFloat(beta_c);
    msg = msg.addFloat(LQR_steer_vect(1));
    msg = msg.addFloat(LQR_steer_vect(2));
    msg = msg.addFloat(accx_offset);
    msg = msg.addFloat(accz_offset);
    msg = msg.addFloat(gyro_offset);
    msg = msg.addFloat(dist_torque_ampl);
    msg = msg.addFloat(omega_notch);
    [numF, denF] = tfdata(filt_torques, 'v');
    msg = msg.addFloat(numF(1));
    msg = msg.addFloat(numF(2));
    msg = msg.addFloat(numF(3));
    msg = msg.addFloat(denF(2));
    msg = msg.addFloat(denF(3));
    msg = msg.addCrcHeadersAndSend(ser);
end