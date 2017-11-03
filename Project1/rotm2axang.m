% In this function, you need to convert the rotation matrix R into axis-angle form

function [axang] = rotm2axang(R)
 
    %% Your code starts here
    theta = acos((trace(R)-1)/2);
    if theta == 0
        vec = [NaN NaN NaN];
    elseif theta == pi
        vec=zeros(2,3);
        theta = [theta;theta];
        rx = sqrt((R(1,1)+1)/2);
        ry = sqrt((R(2,2)+1)/2);
        rz = sqrt((R(3,3)+1)/2);
        if rx==0 && R(2,3) < 0
            vec(1,1:3) = [rx -ry rz];
            vec(2,1:3) = [-rx ry -rz];
        elseif ry==0 && R(1,3) < 0
            vec(1,1:3) = [rx ry -rz];
            vec(2,1:3) = [-rx -ry rz];
        elseif rz==0 && R(1,2) < 0
            vec(1,1:3) = [-rx ry rz];
            vec(2,1:3) = [rx -ry -rz];
        else
            vec(1,1:3) = [rx ry rz];
            vec(2,1:3) = [-rx -ry -rz];
        end
            
    else
        vec = 1/(2*sin(theta))*[R(3,2)-R(2,3) R(1,3)-R(3,1) R(2,1)-R(1,2)];
    end    
 
    axang = [vec, theta];
    %% Your code ends here

end