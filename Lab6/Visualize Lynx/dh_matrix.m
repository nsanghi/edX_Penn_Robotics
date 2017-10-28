function A = dh_matrix(r, alpha, d, theta)

    %% Your code from the first part of this assignment goes here
    %% You can use this function in the lynx_fk function
    rot_z_theta = [cos(theta), -sin(theta), 0, 0; sin(theta), cos(theta), 0, 0; 0,0,1,0; 0,0,0,1];
    trans_z_d = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, d; 0, 0, 0, 1];
    trans_x_r = [1, 0, 0, r; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    rot_x_alpha = [1, 0, 0, 0; 0, cos(alpha), -sin(alpha), 0; 0, sin(alpha), cos(alpha), 0; 0, 0, 0, 1];
    A = rot_z_theta * trans_z_d * trans_x_r * rot_x_alpha;
     
end

