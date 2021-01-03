% -----------------------------------------------------
% -----------------------------------------------------
% Course:   RBE502 Robot Controls
% Authors:  Aniketh Reddy Seelam (aseelam@wpi.edu)
%           Marlon Scott (mscott@wpi.edu)
% Date:     30APR2019
% Title:    RBE502_Term_Project Trajectory Position Control
% 
% -----------------------------------------------------
% Filename: Trajectory_Pos.m
% -----------------------------------------------------

clc
clear all
close all
%% Generate desired trajectory of a square.
% Given target coordinates, this script calculates the trajectory to 
% each point.
%
% The trajectory of the rectangle is as shown below.
%       (c1)%%%%%%%%%%%%%%%%(c2)
%        %%                  %%
%        %%                  %%
%        %%                  %%
%        %%                  %%
%        %%                  %%
%        %%                  %%
%        %%                  %%
%       (c4)%%%%%%%%%%%%%%%%(c3)

global c1 c2 c3 c4
% Corner 1 x, y, z coordinates and end-effector orientation
c1.x=492;
c1.y=0;
c1.z=480;
c1.Ax=-180;
c1.Ay=0;
c1.Az=180;

% Corner 2  x, y, z coordinates and end-effector orientation
c2.x=492;
c2.y=110;
c2.z=480;
c2.Ax=-180;
c2.Ay=0;
c2.Az=180;

% Corner 3  x, y, z coordinates and end-effector orientation
c3.x=330;
c3.y=110;
c3.z=480;
c3.Ax=-180;
c3.Ay=0;
c3.Az=180;

% Corner 4  x, y, z coordinates and end-effector orientation
c4.x=330;
c4.y=0;
c4.z=480;
c4.Ax=-180;
c4.Ay=0;
c4.Az=180;
% Time to start and end at corner 1.
c1.st=0;
c1.et=20;
% Time to start and end at corner 2.
c2.et=5;
c2.st=5;
% Time to start and end at corner 3.
c3.et=10;
c3.st=10;
% Time to start and end at corner 4.
c4.et=15;
c4.st=15;

syms t

global eq_c12 eq_c23 eq_c34 eq_c41
eq_c12 = GetPosEquations(c1,c2);
eq_c23 = GetPosEquations(c2,c3);
eq_c34 = GetPosEquations(c3,c4);
eq_c41 = GetPosEquations(c4,c1);

%% Velocity Kinematics
global J_computed
J_computed = false;

%% Define the IRB robot RigidBodyModel from the URDF
global IRB120
IRB120 = importrobot('irb120.urdf');
IRB120.DataFormat = 'col';
IRB120.Gravity = [0 0 -9.80];

start_config = GetDesiredJointConfig(0);
x0 = deg2rad(start_config(1:12));
tf = 20;
%% ODE45 for ABB IRB 120
global torque
torque = [];

[T,X] = ode45(@(t,x)IRB120_ODE(t,x),[0 tf],x0);
new_X = rad2deg(X);
%% Graphs
figure(1)
plot(T,rad2deg(X(:,1)));
hold on
plot(T,rad2deg(X(:,2)));
hold on
plot(T,rad2deg(X(:,3)));
hold on
plot(T,rad2deg(X(:,4)));
hold on
plot(T,rad2deg(X(:,5)));
hold on
plot(T,rad2deg(X(:,6)));
legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6')
title('Joint Angles(deg) vs Time(sec)');
xlabel('Time(seconds)');
ylabel('Joint Angles(degrees)');
figure(2)
plot(T,rad2deg(X(:,7)));
hold on
plot(T,rad2deg(X(:,8)));
hold on
plot(T,rad2deg(X(:,9)));
hold on
plot(T,rad2deg(X(:,10)));
hold on
plot(T,rad2deg(X(:,11)));
hold on
plot(T,rad2deg(X(:,12)));
legend('$\dot{\theta_1}$','$\dot{\theta_2}$','$\dot{\theta_3}$','$\dot{\theta_4}$','$\dot{\theta_5}$','$\dot{\theta_6}$','Interpreter','latex')
title('Joint Velocities(deg/sec) vs Time(sec)');
xlabel('Time(seconds)');
ylabel('Joint Velocities(in deg/sec)');
figure(3)
plot(T,torque(1,1:size(T,1)));
legend('\tau_1');
title('Control Input Joint 1(Torque(N/mm)) vs Time(sec)');
xlabel('Time(seconds)');
ylabel('Torque(N/m)');
figure(4)
plot(T,torque(2,1:size(T,1)));
legend('\tau_2');
title('Control Input Joint 2(Torque(N/mm)) vs Time(sec)');
xlabel('Time(seconds)');
ylabel('Torque(N/m)');
figure(5)
plot(T,torque(3,1:size(T,1)));
legend('\tau_3');
title('Control Input Joint 3(Torque(N/mm)) vs Time(sec)');
xlabel('Time(seconds)');
ylabel('Torque(N/m)');
figure(6)
plot(T,torque(4,1:size(T,1)));
legend('\tau_4');
title('Control Input Joint 4(Torque(N/mm)) vs Time(sec)');
xlabel('Time(seconds)');
ylabel('Torque(N/m)');
figure(7)
plot(T,torque(5,1:size(T,1)));
legend('\tau_5');
title('Control Input Joint 5(Torque(N/mm)) vs Time(sec)');
xlabel('Time(seconds)');
ylabel('Torque(N/m)');
figure(8)
plot(T,torque(6,1:size(T,1)));
legend('\tau_6');
title('Control Input Joint 6(Torque(N/mm)) vs Time(sec)');
xlabel('Time(seconds)');
ylabel('Torque(N/m)');
x = [];
y = [];
z = [];
e_x=[]; % Expected x
e_y=[]; % Expected y
e_z=[]; % Expected z
for k = 1:size(X)
    pose = FPK_IRB120(new_X(k,1),new_X(k,2),new_X(k,3),new_X(k,4),new_X(k,5),new_X(k,6),0,8);
    x = [x;pose(13)];
    y = [y;pose(14)];
    z = [z;pose(15)];
    e_pos_conf = GetCurrentPositionConfig(T(k));
    e_x = [e_x;e_pos_conf(1)];
    e_y = [e_y;e_pos_conf(2)];
    e_z = [e_z;e_pos_conf(3)];
end
figure(9)
plot3(x,y,z,'r-*');
hold on
plot3(e_x,e_y,e_z,'g--o');
legend('Actual Trajectory','Desired Trajectory')
xlim([200 500])
ylim([-80 180])
zlim([0 500])
xlabel('X(mm)')
ylabel('Y(mm)')
zlabel('Z(mm)')
title('Actual Trajectory vs Desired Trajectory 3D')
figure(10)
plot(x,y,'r-*');
hold on
plot(e_x,e_y,'g--o')
legend('Actual Trajectory','Desired Trajectory')
xlabel('X(mm)')
ylabel('Y(mm)')
title('Actual Trajectory vs Desired Trajectory 2D XY Plane')
%% To send *ALL* Joint Angle Data to RobotStudio for simulation, 
Tsub = diff(T);
Tsub(681)= Tsub(680);

for k = 1:size(X)
    JointAngleData = jointAnglesAndTimeToString(new_X(k,1),...
        new_X(k,2), new_X(k,3), new_X(k,4), new_X(k,5),...
        new_X(k,6), Tsub(k));
    sendAngleStringTimeTCPIP( JointAngleData);
    disp(k)
end
disp('Movement instruction series complete." ');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Utilities
function p = GetCurrentPositionConfig(t)
    global c1 c2 c3 c4
    global eq_c12 eq_c23 eq_c34 eq_c41 
    
    if t>=c1.st && t<c2.et
        p = GetPositionConfig(eq_c12,t);
    elseif t>=c2.st && t<c3.et
        p = GetPositionConfig(eq_c23,t);
    elseif t>=c3.st && t<c4.et
        p = GetPositionConfig(eq_c34,t);
    elseif t>=c4.st && t<=c1.et
        p = GetPositionConfig(eq_c41,t);
    else
        msg = 'Trajectory not available at the time stamp';
        disp(t);
        error(msg)
    end
end

function pos = GetPositionConfig(j_eq,time)
global t
pos = zeros(9,1);
pos(1) = subs(j_eq.x,t,time);
pos(2) = subs(j_eq.y,t,time);
pos(3) = subs(j_eq.z,t,time);
pos(4) = subs(j_eq.xdot,t,time);
pos(5) = subs(j_eq.ydot,t,time);
pos(6) = subs(j_eq.zdot,t,time);
pos(7) = subs(j_eq.xddot,t,time);
pos(8) = subs(j_eq.yddot,t,time);
pos(9) = subs(j_eq.zddot,t,time);
end

function pos_eqs=GetPosEquations(cs,ce)
global t
syms t
coeffsx = coeffs_for_straight_line(cs.x,ce.x,cs.st,ce.et);
coeffsy = coeffs_for_straight_line(cs.y,ce.y,cs.st,ce.et);
coeffsz = coeffs_for_straight_line(cs.z,ce.z,cs.st,ce.et);
pos_eqs.x = coeffsx(1)+coeffsx(2)*t+coeffsx(3)*(t^2)+coeffsx(4)*(t^3);
pos_eqs.y = coeffsy(1)+coeffsy(2)*t+coeffsy(3)*(t^2)+coeffsy(4)*(t^3);
pos_eqs.z = coeffsz(1)+coeffsz(2)*t+coeffsz(3)*(t^2)+coeffsz(4)*(t^3);
pos_eqs.xdot = coeffsx(2)+2*coeffsx(3)*t+3*coeffsx(4)*(t^2);
pos_eqs.ydot = coeffsy(2)+2*coeffsy(3)*t+3*coeffsy(4)*(t^2);
pos_eqs.zdot = coeffsz(2)+2*coeffsz(3)*t+3*coeffsz(4)*(t^2);
pos_eqs.xddot = 2*coeffsx(3)+6*coeffsx(4)*t;
pos_eqs.yddot = 2*coeffsy(3)+6*coeffsy(4)*t;
pos_eqs.zddot = 2*coeffsz(3)+6*coeffsz(4)*t;
end

function q_dot_and_ddot = GetVelocityKinematics(joint_angles,pos)
global t J J_dot J_computed
syms t q1(t) q2(t) q3(t) q4(t) q5(t) q6(t)
if(~J_computed)
    T08=FPK_IRB120(q1(t) ,q2(t), q3(t), q4(t), q5(t), q6(t),0,8);
    %% Position only
    X=[T08(13);T08(14);T08(15)];
    X_dot = diff(X,t);
    syms q1_dot q2_dot q3_dot q4_dot q5_dot q6_dot
    X_d = subs(X_dot,[diff(q1(t), t),diff(q2(t), t),diff(q3(t), t),diff(q4(t), t),diff(q5(t), t),diff(q6(t), t)],[q1_dot,q2_dot,q3_dot,q4_dot,q5_dot,q6_dot]);
    %% Jacobian
    J = [diff(X_d,q1_dot),diff(X_d,q2_dot),diff(X_d,q3_dot),diff(X_d,q4_dot),diff(X_d,q5_dot),diff(X_d,q6_dot)];
    J_dot = diff(J,t);
    J_computed = true;
    disp('Computed the Jacobian for the first time.');
end
J_val = double(subs(J,[q1(t);q2(t);q3(t);q4(t);q5(t);q6(t)],joint_angles));
q_dot = pinv(J_val)*pos(4:6);
J_dot_val = subs(J_dot,[q1(t);q2(t);q3(t);q4(t);q5(t);q6(t);diff(q1(t), t);diff(q2(t), t);diff(q3(t), t);diff(q4(t), t);diff(q5(t), t);diff(q6(t), t)],[joint_angles;q_dot]);
q_ddot = pinv(J_val)*(pos(7:9)-J_dot_val*q_dot);
q_dot_and_ddot = [q_dot;q_ddot];
end

function q = GetDesiredJointConfig(time)
pos = GetCurrentPositionConfig(time);
q = zeros(18,1);
Ax = -180;
Ay = 0;
Az = 180;
q(1:6) = get_IPK_IRB120(pos(1),pos(2),pos(3),Ax,Ay,Az);
q(7:18) = GetVelocityKinematics(q(1:6),pos);
end

function dx = IRB120_ODE(t,x)
    global torque IRB120
    desired_q = GetDesiredJointConfig(t);
    theta_d=deg2rad(desired_q(1:6)); % Desired joint angles
    dtheta_d=deg2rad(desired_q(7:12)); % Desired  joint velocity 
    ddtheta_d=deg2rad(desired_q(13:18)); % Desired joint acceleration 
    
    theta=x(1:6);
    dtheta=x(7:12);
    
    disp('Current Time:');
    disp(t);
    disp('Current Theta:');
    disp(rad2deg(theta));
    disp('Expected Theta:');
    disp(rad2deg(theta_d));
    
    tau = ComputedTorqueControl(theta_d,dtheta_d,ddtheta_d,theta, dtheta); 
    torque =[torque, tau];
 
    dx = zeros(12,1);
    dx(1:6) = x(7:12);
    dx(7:12) = forwardDynamics(IRB120,theta,dtheta,tau);
end


function tau = ComputedTorqueControl(theta_d,dtheta_d,ddtheta_d,theta, dtheta)
    global IRB120
    Kp = 100*eye(6);
    Kv = 10*eye(6);
    e=theta_d-theta; % position error 6x1
    de = dtheta_d - dtheta; % velocity error
    tau = massMatrix(IRB120,theta)*[Kp*e+Kv*de]+inverseDynamics(IRB120,theta,dtheta,ddtheta_d);
end


