function [x_e] = KF_2(y_GPS, FS, HDOP, y_IMU, Ard_T, UTC, Q, R_GPS, R_IMU, R_speed, y_speed)

persistent velLimit
if isempty(velLimit)
    velLimit = 4;
end
persistent accLimit
if isempty(accLimit)
    accLimit = 2;
end

gps_valid_flag = 1;

persistent Ard_T_old
if isempty(Ard_T_old)
    Ard_T_old = Ard_T;
end
T = (Ard_T - Ard_T_old)/1000;
Ard_T_old = Ard_T;

persistent UTC_old
if isempty(UTC_old)
    UTC_old = UTC;
end

persistent x_e_1_N P_1_N
if isempty(x_e_1_N)
    x_e_1_N = [y_GPS(1);0;0];
end
if isempty(P_1_N)
    P_1_N = 1000*eye(3);
end
persistent x_e_1_E P_1_E
if isempty(x_e_1_E)
    x_e_1_E = [y_GPS(2);0;0];
end
if isempty(P_1_E)
    P_1_E = eye(3);
end

%% Dead Reckoning
persistent k_gps
if isempty(k_gps)
    k_gps = 0;
end
persistent GPS_p_x_old GPS_p_y_old
if isempty(GPS_p_x_old)
    GPS_p_x_old = y_GPS(1);
end
if isempty(GPS_p_y_old)
    GPS_p_y_old = y_GPS(2);
end
persistent interval
if isempty(interval)
    interval = 0.1;
end
if UTC == UTC_old || FS == 0 
    gps_valid_flag = 0;
end
UTC_old = UTC;

%% remove bad points
persistent y_IMU_old
if isempty(y_IMU_old)
    y_IMU_old = y_IMU;
end
if y_IMU'*y_IMU > 1
    y_IMU = y_IMU_old;
end
%% select observation vector
if gps_valid_flag == 0
    k_gps = k_gps + 1;
    p_x_dr = GPS_p_x_old + y_speed(1)*interval*k_gps;
    p_y_dr = GPS_p_y_old + y_speed(2)*interval*k_gps;
    y_GPS_IMU_N = [p_x_dr; y_speed(1); y_IMU(1)]; 
    y_GPS_IMU_E = [p_y_dr; y_speed(2); y_IMU(2)];
else
    GPS_p_x_old = y_GPS(1);
    GPS_p_y_old = y_GPS(2);
    y_GPS_IMU_N = [y_GPS(1); y_speed(1); y_IMU(1)];
    y_GPS_IMU_E = [y_GPS(2); y_speed(2); y_IMU(2)];
end
%%
R_GPS = HDOP^2*R_GPS;
if gps_valid_flag == 1
    R_GPS = R_GPS*k_gps^2;
end

Phi = [eye(1), T*eye(1), T^2/2*eye(1);
       zeros(1), eye(1), T*eye(1);
       zeros(1), zeros(1), eye(1)]; % transfer matrix
Sigma = [T^3/6*ones(1,1);T^2/2*ones(1,1);T*ones(1,1)];
persistent H_GPS_IMU
if isempty(H_GPS_IMU)
    H_GPS_IMU = eye(3);
end
%% kf --north
R_GPS_IMU_N = [R_GPS(1,1),  0,  0;
              0,   R_speed,  0;
              0,   0,    R_IMU];
Q_N = Q;
%--------------- 1 step prediction ----------------
x_1_p_N = Phi*x_e_1_N;
P_1_p_N = Phi*P_1_N*Phi' + Sigma*Q_N*Sigma';
%--------------- innovation------------------
%GPS_IMU
epsilon_GPS_IMU_N = y_GPS_IMU_N - H_GPS_IMU*x_1_p_N;
Q_ep_GPS_IMU_N = H_GPS_IMU*P_1_p_N*H_GPS_IMU' + R_GPS_IMU_N;
K_GPS_IMU_N = P_1_p_N*H_GPS_IMU'*Q_ep_GPS_IMU_N^(-1);

%--------------------estimate------------------------
x_e_N = x_1_p_N + K_GPS_IMU_N*epsilon_GPS_IMU_N;
P_N = (eye(3) - K_GPS_IMU_N*H_GPS_IMU)*P_1_p_N;   

%% kf --east
R_GPS_IMU_E = [R_GPS(2,2),  0,  0;
              0,   R_speed,  0;
              0,   0,    R_IMU];
Q_E = Q;
%--------------- 1 step prediction ----------------
x_1_p_E = Phi*x_e_1_E;
P_1_p_E = Phi*P_1_E*Phi' + Sigma*Q_E*Sigma';
%--------------- innovation------------------
%GPS_IMU
epsilon_GPS_IMU_E = y_GPS_IMU_E - H_GPS_IMU*x_1_p_E;
Q_ep_GPS_IMU_E = H_GPS_IMU*P_1_p_E*H_GPS_IMU' + R_GPS_IMU_E;
K_GPS_IMU_E = P_1_p_E*H_GPS_IMU'*Q_ep_GPS_IMU_E^(-1);

%--------------------estimate------------------------
x_e_E = x_1_p_E + K_GPS_IMU_E*epsilon_GPS_IMU_E;
P_E = (eye(3) - K_GPS_IMU_E*H_GPS_IMU)*P_1_p_E;   

%%

if gps_valid_flag == 1
    k_gps = 0;
end
%% avoid steep change
persistent bad_point_count
if isempty(bad_point_count)
    bad_point_count = 0;
end
if (sqrt( (x_e_N(1)-x_e_1_N(1))^2 + (x_e_E(1)-x_e_1_E(1))^2 )/interval > velLimit)  %velocity > velLimit m/s
% if ((x_e_N(2)-x_e_1_N(2))/0.1 > accLimit || (x_e_E(2)-x_e_1_E(2))/0.1 > accLimit)
%     p_x_dr = x_e_1_N(1) + y_speed(1)*interval;
%     p_y_dr = x_e_1_E(1) + y_speed(2)*interval;
    x_e = [x_e_1_N(1);x_e_1_E(1);x_e_1_N(2);x_e_1_E(2);x_e_1_N(3);x_e_1_E(3)];
    bad_point_count = bad_point_count+1;
    display(bad_point_count)
%     display(x_e_1)
    display(T)
else
    x_e_1_N = x_e_N;
    P_1_N = P_N; 
    x_e_1_E = x_e_E;
    P_1_E = P_E; 
    x_e = [x_e_N(1);x_e_E(1);x_e_N(2);x_e_E(2);x_e_N(3);x_e_E(3)];
end
end

