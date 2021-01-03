% -----------------------------------------------------
% -----------------------------------------------------
% Course:   RBE502 Robot Controls
% Authors:  Aniketh Reddy Seelam (aseelam@wpi.edu)
%           Marlon Scott (mscott@wpi.edu)
% Date:     30APR2019
% Title:    6DOF Robot EE Orientation Calculator
% 
% -----------------------------------------------------
% Filename: Orientation.m (MATLAB function)
% -----------------------------------------------------

function [Ax,Ay,Az]=Orientation(T)
    if (T(3)~=1 && T(3)~=-1)
        Ay=-asind(T(3));
        Ax=atan2d(T(7)/cosd(Ay),T(11)/cosd(Ay));
        Az=atan2d(T(2)/cosd(Ay),T(1)/cosd(Ay));
    else
        Az=0;
    if T(3)==-1
        Ay=90;
        Ax=Az+atan2d(T(5),T(9));
    else
        Ay=-90;
        Ax=-Az+atan2d(-T(5),-T(9));
    end
    end
end