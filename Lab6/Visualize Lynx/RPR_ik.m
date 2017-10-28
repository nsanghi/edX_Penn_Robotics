function [ ik_sol ] = RPR_ik( x, y, z, R )
%RPR_IK Write your code here. The input to the function will be the position of
%    the end effector (in inches) in the world frame, and the 
%    Rotation matrix R_30 as described in the question.
%    The output must be the joint angles and extensions of the robot to achieve 
%    the end effector position and orientation.

    %% YOUR CODE GOES HERE
    
    ik_sol=[];
    if abs(R(1,1)^2+R(2,1)^2-1) < 1e-5 
        theta1 = atan2(R(2,1),R(1,1));
        if abs(R(3,2)^2+R(3,3)^2-1) < 1e-5
            theta3 = atan2(R(3,3),-R(3,2));
            d2 = (10+5*R(3,3)-z)*sqrt(2);
            ik_sol=[theta1, d2, theta3];
        end
    end

end

