% -----------------------------------------------------
% -----------------------------------------------------
% Course:   RBE502 Robot Controls
% Authors:  Aniketh Reddy Seelam (aseelam@wpi.edu)
%           Marlon Scott (mscott@wpi.edu)
% Date:     30APR2019
% Title:    Inverse Kinematics Solution for the ABB IRB 120 Robot
% 
% -----------------------------------------------------
% Filename: IPK_IRB120.m (MATLAB function)
% -----------------------------------------------------

function final_q = IPK_IRB120(T07)
    q = zeros(4,6);
    d1=290;d4=302;a2=270;a3=70;
    x = T07(13);
    y = T07(14);
    z = T07(15);
    psi = atan2d(z-d1,hypot(x,y));
    q(1,1) = atan2d(y,x);
    q(2,1) = q(1,1);
    r1=hypot(hypot(x,y),z-d1);
    r2=hypot(a3,d4);
    D1 = (r1^2+a2^2-r2^2)/(2*a2*r1);
    beta = atand(a3/d4);
    D2 = (r1^2+r2^2-a2^2)/(2*r1*r2);
    
    % Elbow Up Front Conf
    q(1,2) = 90-psi-acosd(D1);
    gamma = acosd(D2)-psi;
    q(1,3)=beta+gamma-q(1,2);
    
    % Elbow Down Front Conf
    q(2,2) = 90-psi+acosd(D1);
    gamma = -acosd(D2)-psi;
    q(2,3)=beta+gamma-q(2,2);
    q(3,1:3) = q(1,1:3);
    q(4,1:3) = q(2,1:3);

    R07 = T07(1:3,1:3);

    [q(1,4),q(1,5),q(1,6)]=GetWristAngles(q(1,1),q(1,2),q(1,3),R07,1);
    [q(2,4),q(2,5),q(2,6)]=GetWristAngles(q(2,1),q(2,2),q(2,3),R07,1);
    [q(3,4),q(3,5),q(3,6)]=GetWristAngles(q(3,1),q(3,2),q(3,3),R07,2);
    [q(4,4),q(4,5),q(4,6)]=GetWristAngles(q(4,1),q(4,2),q(4,3),R07,2);
    final_q = q(1,:);
end

function [q4,q5,q6] = GetWristAngles(q1,q2,q3,R07,sol_type)
    T04 = FPK_IRB120(q1,q2,q3,0,0,0,0,4);
    R04 = T04(1:3,1:3);
    R57 = transpose(R04)*R07;

    if(sol_type==1)
        q5 = atan2d(hypot(R57(3),R57(6)),R57(9));
    elseif(sol_type==2)
        q5 = atan2d(-hypot(R57(3),R57(6)),R57(9));
    end
    if abs(q5-0)<0.01
        q4=0;
        q6=atan2d(R57(4),-R57(1));
    elseif (q5==180 || q5==-180)
    q4=0;
    q6=atan2d(-R57(4),R57(1));
    else
        q6 = atan2d(R57(6)/sind(q5),-R57(3)/sind(q5));
        q4 = atan2d(-R57(8)/sind(q5),-R57(7)/sind(q5));
    end

end