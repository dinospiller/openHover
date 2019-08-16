%% IMU model
clear all;
Ts=0.01;
A_imu=[0 1; 0 0]
B_imu=[1;0]
C_imu=[1 0]
D_imu=0

sys_imu=ss(A_imu,B_imu,C_imu,D_imu)
sys_imu_d=c2d(sys_imu,Ts)