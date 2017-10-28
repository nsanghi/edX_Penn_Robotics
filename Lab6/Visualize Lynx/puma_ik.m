function [ ik_sol ] = puma_ik( x, y, z, R )
%PUMA_IK Write your code here. The input to the function will be the position of
%    the end effector (in inches) in the world frame, and the 
%    Rotation matrix R_60 as described in the question.
%    The output must be the joint angles of the robot to achieve 
%    the desired end effector position and orientation.

    %% YOUR CODE GOES HERE
    a = 13;
    b = 2.5;
    c = 8.0;
    d = 2.5;
    e = 8.0;
    f = 2.5;
    x = x - f*R(1,3);
    y = y - f*R(2,3);
    z = z - f*R(3,3);
    
    s3 = (x^2 + y^2 - (b+d)^2 + (z-a)^2 - c^2 - e^2)/(-2*c*e);
    ik_sol = [];
    if abs(s3)<= 1
        theta3 = asin(s3);
        if abs(theta3) < 1e-25
            theta3=0;
        end
        if abs(z-a)>1e-5
            p = (e * cos(theta3))/(z-a);
            q = (c - e*sin(theta3))/(z-a);
            r = sqrt(p*p+q*q);
            s = 1/r;
        else
            p = (e * cos(theta3));
            q = (c - e*sin(theta3));
            r = sqrt(p*p+q*q);
            s = 0;
        end
        if (abs(s)<=1) 
            alpha = atan2(q,p);
            beta = acos(s);
            theta2 = -beta+alpha;
            if abs(theta2) < 1e-25
                theta2=0;
            end
            
            if abs(y)>1e-5
                p = (b+d)/y;
                q = (c*cos(theta2) - e* sin(theta2 + theta3))/y;
                r = sqrt(p*p+q*q);
                s = 1/r;
            else
                p = (b+d);
                q = (c*cos(theta2) - e* sin(theta2 + theta3));
                r = sqrt(p*p+q*q);
                s = 0;
            end
            if (abs(s)<=1)
                alpha = atan2(q,p);
                beta = acos(s);
                theta1 = -beta+alpha;
                if abs(theta1) < 1e-25
                    theta1=0;
                end
                
                R03 = [cos(theta1)*cos(theta2+theta3), -sin(theta1), -cos(theta1)*sin(theta2+theta3); 
                       sin(theta1)*cos(theta2+theta3), cos(theta1), -sin(theta1)*sin(theta2+theta3);
                       sin(theta2+theta3), 0, cos(theta2+theta3)];
                det(R03)
                R36 = R03'*R;
                
                
                theta5 = -acos(R36(3,3));
                if abs(theta5) < 1e-25
                    theta5=0;
                end
                if sin(theta5)<=0
                    theta4 = atan2(R36(2,3), R36(1,3));
                    theta6 = atan2(R36(3,2), -R36(3,1));
                else
                    theta4 = atan2(-R36(2,3), -R36(1,3));
                    theta6 = atan2(-R36(3,2), R36(3,1));
                end
                if abs(theta4) < 1e-25
                    theta4=0;
                end
                if abs(theta6) < 1e-25
                    theta6=0;
                end
                ik_sol = [theta1 theta2 theta3 theta4 theta5 theta6];
                
            end
        end
            
    end
    

end