function [ pos ] = lynx_fk( theta1, theta2, theta3, theta4, theta5, g )
%LYNX_FK The input to the function will be the joint
%    angles of the robot in radians, and the distance between the gripper pads in inches.
%    The output must contain 10 positions of various points along the robot arm as specified
%    in the question.

    %% YOUR CODE GOES HERE
    T_0_1 = compute_dh_matrix(0, -pi/2, 3, theta1);
    T_1_2 = compute_dh_matrix(5.75, 0, 0, -pi/2+theta2);
    T_2_3 = compute_dh_matrix(7.375, 0, 0, +pi/2+theta3);
    T_3_4 = compute_dh_matrix(0, -pi/2, 0, -pi/2+theta4);
    T_4_5 = compute_dh_matrix(0, 0, 4.125, theta5);
    T_0_2 = T_0_1 * T_1_2;
    T_0_3 = T_0_2 * T_2_3;
    T_0_4 = T_0_3 * T_3_4;
    T_0_5 = T_0_4 * T_4_5;
    
    world = [0;0;0];
    frame_1 = T_0_1(1:3,4);
    frame_2 = T_0_2(1:3,4);
    frame_3 = T_0_3(1:3,4);
    frame_4 = T_0_4(1:3,4);

    pt1 = T_0_5 * [0;0; -1.125; 1];
    pt2 = T_0_5 * [g/2;0; -1.125; 1];
    pt3 = T_0_5 * [-g/2;0; -1.125; 1];
    pt4 = T_0_5 * [g/2;0; 0; 1];
    pt5 = T_0_5 * [-g/2;0; 0; 1];
    
    pt1 = pt1(1:3);
    pt2 = pt2(1:3);
    pt3 = pt3(1:3);
    pt4 = pt4(1:3);
    pt5 = pt5(1:3);
    
    pos = [world frame_1 frame_2 frame_3 frame_4 pt1 pt2 pt3 pt4 pt5];
    pos = pos';

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