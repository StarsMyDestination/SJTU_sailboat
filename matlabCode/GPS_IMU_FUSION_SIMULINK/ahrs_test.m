load('16_12_04_ahrs_test.mat')
%%
a_x = arduinoData_Ax;
a_y = arduinoData_Ay;
a_z = arduinoData_Az;
phi = arduinoData_roll;
theta = arduinoData_pitch;
psi = arduinoData_yaw;
%% section1
a_x = a_x(1200:2200);
a_y = a_y(1200:2200);
a_z = a_z(1200:2200);
phi = phi(1200:2200);
theta = theta(1200:2200);
psi = psi(1200:2200);
%% section2
a_x = a_x(2500:4000);
a_y = a_y(2500:4000);
a_z = a_z(2500:4000);
phi = phi(2500:4000);
theta = theta(2500:4000);
psi = psi(2500:4000);
%% plot
close all
figure (1)
subplot(3,1,1);plot(a_x);title('a_x')
subplot(3,1,2);plot(a_y);title('a_y')
subplot(3,1,3);plot(a_z);title('a_z')
figure (2)
subplot(3,1,1);plot(phi);title('phi')
subplot(3,1,2);plot(theta);title('theta')
subplot(3,1,3);plot(psi);title('psi')
%% coord trans
len = length(a_x);
a_x_ned = zeros(len, 1);
a_y_ned = zeros(len, 1);
a_z_ned = zeros(len, 1);
for n = 1:len
    a_v_ned = coord_Trans(phi(n), theta(n), psi(n), a_x(n), a_y(n), a_z(n));
    a_x_ned(n) = a_v_ned(1);
    a_y_ned(n) = a_v_ned(2);
    a_z_ned(n) = a_v_ned(3)+10;
end
%% plot
close all
figure (3)
subplot(3,1,1);plot(a_x_ned);title('a_x')
subplot(3,1,2);plot(a_y_ned);title('a_y')
subplot(3,1,3);plot(a_z_ned);title('a_z')
a_x_m = mean(a_x_ned)
a_y_m = mean(a_y_ned)
a_z_m = mean(a_z_ned)
%%
Ard_T = 0; x_e_1 = zeros(9,1); P_1 = eye(9);
UTC = 0;
for n = 1:len
    Ard_T = Ard_T+100;
    if mod(n,5) == 0
        UTC = UTC + 0.5;
    end
    y_IMU = [a_x_ned(n);a_y_ned(n);a_z_ned(n)];
    [x,P] = KF([0;0;0],1,1.1,y_IMU,Ard_T,UTC,0.01,eye(3),0.01*eye(3),x_e_1, P_1)
    x_e_1 = x;
    P_1 = P;
end
    
