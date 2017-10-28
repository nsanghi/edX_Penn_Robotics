function [ pos, R ] = RPR_fk( theta1, d2, theta3 )
%RPR_FK Write your code here. The input to the function will be the joint
%    angles of the robot in radians, and the extension of the prismatic joint in inches.
%    The output includes: 
%    1) The position of the end effector and the position of 
%    each of the joints of the robot, as explained in the question.
%    2) The rotation matrix R_03, as explained in the question.

    %% YOUR CODE GOES HERE
    
    frame_0 = [0,0,0];
    frame_1 = [0,0,10];
    R1 = [cos(theta1), -sin(theta1), 0; sin(theta1), cos(theta1), 0; 0,0,1];
    frame_2 = R1*[0;d2*sin(pi/4); 10-d2*cos(pi/4)];
    frame_2 = frame_2';
    frame_3 = R1*[0;d2*sin(pi/4)+5*cos(theta3); 10-d2*cos(pi/4)+5*sin(theta3)];
    frame_3 = frame_3';

    
    pos = [frame_0; frame_1; frame_2; frame_3];
    R = R1*[1, 0, 0; 0, cos(theta3-pi/2), -sin(theta3-pi/2); 0, sin(theta3-pi/2), cos(theta3-pi/2)];
    
    

end