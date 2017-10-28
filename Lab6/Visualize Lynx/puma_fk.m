function [pos, R] = puma_fk(theta1, theta2, theta3, theta4, theta5, theta6)
%PUMA_FK The input to the function will be the joint angles of the robot in radians.
%    The output must contain end effector position of the robot arm and the rotation matrix representing the rotation from frame
%    6 to frame 0, as specified in the question.

    %% Your code goes here
    T_0_1 = compute_dh_matrix(0, pi/2, 13, theta1);
    T_1_2 = compute_dh_matrix(8, 0, -2.5, theta2);
    T_2_3 = compute_dh_matrix(0, -pi/2, -2.5, theta3);
    T_3_4 = compute_dh_matrix(0, pi/2, 8, theta4);
    T_4_5 = compute_dh_matrix(0, -pi/2, 0, theta5);
    T_5_6 = compute_dh_matrix(0, 0, 2.5, theta6);
    T_0_2 = T_0_1 * T_1_2;
    T_0_3 = T_0_2 * T_2_3;
    T_0_4 = T_0_3 * T_3_4;
    T_0_5 = T_0_4 * T_4_5;
    T_0_6 = T_0_5 * T_5_6;
    
    pos = T_0_6(1:3,4)';    
    R = T_0_6(1:3, 1:3);
    
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

