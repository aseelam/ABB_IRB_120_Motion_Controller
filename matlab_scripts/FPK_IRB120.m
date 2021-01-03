% -----------------------------------------------------
% -----------------------------------------------------
% Course:   RBE502 Robot Controls
% Authors:  Aniketh Reddy Seelam (aseelam@wpi.edu)
%           Marlon Scott (mscott@wpi.edu)
% Date:     30APR2019
% Title:    Forward Kinematics for the ABB IRB 120 Robot
% 
% -----------------------------------------------------
% Filename: FPK_IRB120.m (MATLAB function)
% -----------------------------------------------------

function Tmn = FPK_IRB120(q1,q2,q3,q4,q5,q6,m,n)
    %% The frames of ABB IRB 120 Robot used to compute the Forward Position and Velocity Kinematics.
    % <<frames.png>>
    %% The DH parameter table and the variables for each of the 4 parameters are obtained as shown.
    % <<dhtable.jpg>>
    theta=[q1;q2-90;q3;0;q4;q5;q6+180;0]; % Initializing an array of size 6x1 for theta.
    d=[290;0;0;302;0;0;0;72];     % Initializing an array of size 6x1 for d.
    a=[0;270;70;0;0;0;0;0];     % Initializing an array of size 6x1 for a.
    alpha=[-90;0;-90;0;90;-90;0;0]; % Initializing an array of size 6x1 for alpha.
    %% Frame Transformations
    % The frame transformations from one link to the next as homogeneous
    % transformations matrices are determined. The MATLAB function is created
    % and is used in a loop to generate intermediate transfromation matrices. 
    temp_T = eye(4); % Initializing to an identity matrix.
    for i=m+1:n
        variable.(strcat('T',num2str(i-1),num2str(i)))= dhparam2matrix(theta(i), d(i), a(i), alpha(i));
        temp_T= temp_T*variable.(strcat('T',num2str(i-1),num2str(i)));
    end
    Tmn = temp_T;
%% Functions
    function T = dhparam2matrix(theta, d, a, alpha)
        T=Rotz(theta)*Transz(d)*Transx(a)*Rotx(alpha);
    end

    function Rx=Rotx(alpha)
        Rx=[1 0 0 0; 0 cosd(alpha) -sind(alpha) 0;0 sind(alpha) cosd(alpha) 0;0 0 0 1];
    end

    function Tx=Transx(a)
        Tx=[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
    end

    function Rz=Rotz(theta)
        Rz=[cosd(theta) -sind(theta) 0 0;sind(theta) cosd(theta) 0 0;0 0 1 0;0 0 0 1];
    end

    function Tz=Transz(d)
        Tz=[1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];
    end

end