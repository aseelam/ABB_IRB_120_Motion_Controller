% -----------------------------------------------------
% -----------------------------------------------------
% Course:   RBE502 Robot Controls
% Authors:  Aniketh Reddy Seelam (aseelam@wpi.edu)
%           Marlon Scott (mscott@wpi.edu)
% Date:     30APR2019
% Title:    TCPIP string transmission function
%           This function transmits string data via TCPIP to a server
%           with the TCPIIP address shown
% -----------------------------------------------------
% Filename: sendAngleStringTimeTCPIP.m (MATLAB function)
% -----------------------------------------------------

function sendAngleStringTimeTCPIP( X)

    tc=tcpip('127.0.0.1',55000);
    
    %Opening the communication
    fopen(tc);   
    
    fwrite(tc, X);
    
    % serverMessage = fread(tc); % Moved to location after data is sent    

end