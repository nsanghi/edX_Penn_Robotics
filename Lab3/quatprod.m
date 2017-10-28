function q=quatprod(q1,q2)
    % All quaternions q, q1 and q2 are represented as 1-by-4 row vectors 
    q=zeros(1,4);
    
    %%%%% your code starts here %%%%%
    crs = cross(q1(2:4),q2(2:4));
    q(1)=q1(1)*q2(1)-q1(2:4)*q2(2:4)';
    q(2)=q1(1)*q2(2)+q2(1)*q1(2)+crs(1);
    q(3)=q1(1)*q2(3)+q2(1)*q1(3)+crs(2);
    q(4)=q1(1)*q2(4)+q2(1)*q1(4)+crs(3);
    %%%%% your code ends here %%%%%
    
end

