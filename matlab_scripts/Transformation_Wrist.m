% -----------------------------------------------------
% -----------------------------------------------------
% Course:   RBE502 Robot Controls
% Authors:  Aniketh Reddy Seelam (aseelam@wpi.edu)
%           Marlon Scott (mscott@wpi.edu)
% Date:     30APR2019
% Title:    Calculates the transformation to the wrist 
%           given the end effector pose.
% 
% -----------------------------------------------------
% Filename: Transformation_Wrist.m (MATLAB function)
% -----------------------------------------------------

function T_0_wrist = Transformation_Wrist(x,y,z,Ax,Ay,Az)
    % ZYX Euler Angle Rotation. This is the final rotation matrix of IRB120.
    R_0_tip = Rotz(Az)*Roty(Ay)*Rotx(Ax);
    % The translation from base to the tip.
    t_0_tip = [x;y;z];
    % The transformation from base to the tip.
    T_0_tip = [R_0_tip,t_0_tip;0,0,0,1];
    % The transformation from the wrist to the base. Pure translation along z.
    T_wrist_tip = FPK_IRB120(0,0,0,0,0,0,7,8);
    % The transformation from the base to wrist used for IPK.
    T_0_wrist = T_0_tip/T_wrist_tip;
    
    %% 3x3 Rotation Matrices
    function R = Rotz(a)
        R =[cosd(a),-sind(a),0;sind(a),cosd(a),0;0,0,1];
    end

    function R = Roty(a)
        R =[cosd(a),0,sind(a);0,1,0;-sind(a),0,cosd(a)];
    end

    function R = Rotx(a)
        R =[1,0,0;0,cosd(a),-sind(a);0,sind(a),cosd(a)];
    end
end