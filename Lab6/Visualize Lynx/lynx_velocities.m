function [ v05, w05 ] = lynx_velocities( thetas, thetadot )
%LYNX_VELOCITIES The input to the function will be:
%    thetas: The joint angles of the robot in radians - 1x5 matrix
%    thetadot: The rate of change of joint angles of the robot in radians/sec - 1x5 matrix
%    The output has 2 parts:
%    v05 - The linear velocity of frame 5 with respect to frame 0, expressed in frame 0.
%    w05 - The angular velocity of frame 5 with respect to frame 0, expressed in frame 0.
%    They are both 1x3 matrices of the form [x y z] for a vector xi + yj + zk

    %% YOUR CODE GOES HERE
    
    T_0_1 = compute_dh_matrix(0, -pi/2, 3, thetas(1));
    T_1_2 = compute_dh_matrix(5.75, 0, 0, -pi/2+thetas(2));
    T_2_3 = compute_dh_matrix(7.375, 0, 0, +pi/2+thetas(3));
    T_3_4 = compute_dh_matrix(0, -pi/2, 0, -pi/2+thetas(4));
    T_4_5 = compute_dh_matrix(0, 0, 4.125, thetas(5));
    T_0_2 = T_0_1 * T_1_2;
    T_0_3 = T_0_2 * T_2_3;
    T_0_4 = T_0_3 * T_3_4;
    T_0_5 = T_0_4 * T_4_5;
    P_0 = [0;0;0];
    P_1 = T_0_1(1:3,4);
    P_2 = T_0_2(1:3,4);
    P_3 = T_0_3(1:3,4);
    P_4 = T_0_4(1:3,4);
    P_5 = T_0_5(1:3,4);
    z0  = [0;0;1];
    z1  = T_0_1(1:3,3);
    z2  = T_0_2(1:3,3);
    z3  = T_0_3(1:3,3);
    z4  = T_0_4(1:3,3);
    z5  = T_0_5(1:3,3);
    Jv1 = cross(z0,P_5 - P_0);
    Jv2 = cross(z1,P_5 - P_1);
    Jv3 = cross(z2,P_5 - P_2);
    Jv4 = cross(z3,P_5 - P_3);
    Jv5 = cross(z4,P_5 - P_4);
    J = [Jv1 Jv2 Jv3 Jv4 Jv5; z0 z1 z2 z3 z4];
    
    V = J * thetadot';
    
    v05 = V(1:3)';
    w05 = V(4:6)';
    
end


function A = compute_dh_matrix(r, alpha, d, theta)

    %% Your code from the first part of this assignment goes here
    %% You can use this function in the lynx_fk function
    rot_z_theta = [cos(theta), -sin(theta), 0, 0; sin(theta), cos(theta), 0, 0; 0,0,1,0; 0,0,0,1];
    trans_z_d = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, d; 0, 0, 0, 1];
    trans_x_r = [1, 0, 0, r; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    rot_x_alpha = [1, 0, 0, 0; 0, cos(alpha), -sin(alpha), 0; 0, sin(alpha), cos(alpha), 0; 0, 0, 0, 1];
    A = rot_z_theta * trans_z_d * trans_x_r * rot_x_alpha;
     
end

