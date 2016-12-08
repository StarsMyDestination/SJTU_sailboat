function [x_e, P] = KF(y_GPS, FS, HDOP, y_IMU, Ard_T, UTC, Q, R_GPS,R_IMU, x_e_1, P_1)
y_GPS_IMU = [y_GPS; y_IMU];

gps_valid_flag = 0;

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
if UTC ~= UTC_old && FS ~= 0
    gps_valid_flag = 1;
end
UTC_old = UTC;

R_GPS = HDOP^2*R_GPS;
R_GPS_IMU = [R_GPS, zeros(3);
             zeros(3), R_IMU];

%% kf
Phi = [eye(3), T*eye(3), T^2/2*eye(3);
       zeros(3), eye(3), T*eye(3);
       zeros(3), zeros(3), eye(3)]; % transfer matrix
Sigma = [T^3/6*ones(3,1);T^2/2*ones(3,1);T*ones(3,1)];
persistent H_GPS_IMU H_IMU
if isempty(H_GPS_IMU)
    H_GPS_IMU = [eye(3),zeros(3,6);
            zeros(3,6), eye(3)];
end
if isempty(H_IMU)
    H_IMU = [zeros(3,6),eye(3)];
end

% 1 step prediction
x_1_p = Phi*x_e_1;
P_1_p = Phi*P_1*Phi' + Sigma*Q*Sigma';
% innovation
epsilon_IMU = y_IMU - H_IMU*x_1_p;
Q_ep_IMU = H_IMU*P_1_p*H_IMU' + R_IMU;
K_IMU = P_1_p*H_IMU'*Q_ep_IMU^(-1);
if gps_valid_flag == 0
    %estimate
    x_e = x_1_p + K_IMU*epsilon_IMU;
    P = (eye(9) - K_IMU*H_IMU)*P_1_p;    
else
    % innovation
    epsilon_GPS_IMU = y_GPS_IMU - H_GPS_IMU*x_1_p;
    Q_ep_GPS_IMU = H_GPS_IMU*P_1_p*H_GPS_IMU' + R_GPS_IMU;
    K_GPS_IMU = P_1_p*H_GPS_IMU'*Q_ep_GPS_IMU^(-1);
    % estimate
    x_e = x_1_p + K_GPS_IMU*epsilon_GPS_IMU;
    P = (eye(9) - K_GPS_IMU*H_GPS_IMU)*P_1_p;    
end

end
