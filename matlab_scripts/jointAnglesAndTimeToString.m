% -----------------------------------------------------
% -----------------------------------------------------
% Course:   RBE502 Robot Controls
% Authors:  Aniketh Reddy Seelam (aseelam@wpi.edu)
%           Marlon Scott (mscott@wpi.edu)
% Date:     30APR2019
% Title:    6DOF Robot Configuration String Function
%           This function formats and concatenates the joint angle
%           and movement time data for a 6DOF robot, for transmission
%           specifically to a RobotStudio RAPID program.
%           The receiving program of this function's output must be
%           formated to receive this data.
% -----------------------------------------------------
% Filename: jointAnglesAndTimeToString.m (MATLAB function)
% -----------------------------------------------------

function jointData = jointAnglesAndTimeToString(j1, j2, j3, j4, j5, j6, t)
    
    
    j1str = num2str(j1, '%08.4f');
    j2str = num2str(j2, '%08.4f');
    j3str = num2str(j3, '%08.4f');
    j4str = num2str(j4, '%08.4f');
    j5str = num2str(j5, '%08.4f');
    j6str = num2str(j6, '%08.4f');
    t_str = num2str(t, '%04.2f');
    jointData = strcat(j1str, j2str, j3str, j4str, j5str, j6str, t_str);  

end