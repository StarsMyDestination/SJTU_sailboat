function a_v_ned = coord_Trans(roll, pitch, yaw, a_x, a_y, a_z)
phi = roll*pi/180;
theta = pitch*pi/180;
psi = yaw*pi/180;
a_v = [a_x, a_y, a_z]';
J_trans = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
    sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
    -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
a_v_ned = J_trans*a_v;