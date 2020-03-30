% *** This script is the final implementation of the project *** %
% *** Project website: 
% http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone

% Including : a (screw or straight line motion only)trajectory generater
%             a kinematic simulator to update current configuration
%             (first-order Euler-step method)
%             a feedforward and PI feedbackward velocity controller in the
%             task space.
% Introduction : This script use the two Class file "mobileRobot.m" and "
%                youbot.m" which is inherited from its superclass to 
%                conclude and generate all the useful function and 
%                parameter needed. 
%                The code first generate the end-effector trajectory then 
%                control the youbot to follow that trajectory from a
%                differnt initial configuration.

root = fileparts(pwd);
project_name = "Final Project";
sourceCode_name = "mr";
addpath(genpath(fullfile(root,project_name,sourceCode_name)));

% mobile base parameters
l = 0.47/2;
w = 0.3/2;
r = 0.0475;
T_b0 = RpToTrans(eye(3),[0.1662,0,0.0026]');
M_0e = RpToTrans(eye(3),[0.033,0,0.6546]');
Blists = [[0,0,1,0,0.033,0]',[0,-1,0,-0.5076,0,0]',[0,-1,0,-0.3526,0,0]'...
    [0,-1,0,-0.2176,0,0]',[0,0,1,0,0,0]'];

% cube configuration
T_sc_initial = RpToTrans(eye(3),[1,0,0.025]');
T_sc_goal = RpToTrans(rotz(-pi/2),[0,-1,0.025]');
a = pi/5;
T_ce_standoff = [[-sin(a),0,-cos(a),0]',[0,1,0,0]',[cos(a),0,-sin(a),0]',...
    [0,0,0.25,1]'];
T_ce_grasp = [[-sin(a),0,-cos(a),0]',[0,1,0,0]',[cos(a),0,-sin(a),0]',...
    [0,0,0,1]'];
% end-effector planned configuration(reference) 
T_se_initial = [0,0,1,0;0,1,0,0;-1,0,0,0.5;0,0,0,1];
T_standoff_initial = T_sc_initial * T_ce_standoff;
T_grasp = T_sc_initial * T_ce_grasp;
T_standoff_final = T_sc_goal * T_ce_standoff;
T_release = T_sc_goal * T_ce_grasp;

%Construct a cell array for the path
T_configure = {T_se_initial,T_standoff_initial,T_grasp,T_grasp,...
    T_standoff_initial,T_standoff_final,T_release,T_release,...
    T_standoff_final};
disp('Initialization completed');

% Create the youbot object Mybot
Mybot = youbot(l,w,r,T_b0,M_0e,Blists);
disp('Youbot object created!');
% Generating reference trajectory
dt = 0.01;% manual choose 
T_total = 13;% manual choose the total motion time 
Tf = calculateTf(T_total);% calculate the weighted time for each piece
Traj = [];% N * 13 matrix, N is the number of reference frame
grasp_state = 0;
for i = 1:8
    if i == 3 
        grasp_state = 1;
    elseif i == 7
        grasp_state = 0;
    end
    
    Trajectory = Mybot.TrajectoryGenerator(T_configure{i},...
        T_configure{i+1},Tf(i),dt,grasp_state,'Cartesian',5);
    Traj = [Traj;Trajectory];
end
writematrix(Traj,'Traj.csv');
disp('Trajectory generated');

% Manual choose the initial parameter of the controller and the robot
% configuration
Mybot.q = [0,0,0];
Mybot.theta = [0,0,0.2,-1.67,0]';
Mybot.wheelAngle = [pi/2,pi/2,pi/2,pi/2];% Do not affect the simulation
Mybot.kp = 1.5 * eye(6);
Mybot.ki = 0 * eye(6);
maxspeed = 12.3*ones(1,9);% take uniform max speed.
jointLimits = [[pi,-pi]',[pi,-pi]',...
    [pi,-pi]',[pi,-pi]'...
    ,[pi,-pi]'];% Designed values to avoid collision : [max;min]

[Td,grasp] = traj2mat(Traj);% Td is a 3D matrix

% Apply Fb control to the Youbot object.
% All other parameter needed is included in the obj's property.
[Animation,Xerr] = Mybot.FeedbackControl(dt,Td,maxspeed,grasp,jointLimits);
disp('Feedback Control applied, Animation file generated');

% plot the error twist between the reference configuration and current
% configurtaion: to see the control system performance.
p = plot(Xerr','LineWidth',1.5);
title('Xerr versus time','FontSize',12,'FontWeight','bold')
xlabel('Time (0.01s)','FontSize',12,'FontWeight','bold');
ylabel('Error Twist','FontSize',12,'FontWeight','bold');
legend(p,'Wx','Wy','Wz','Vx','Vy','Vz')
grid on;
writematrix(Animation,'Animation.csv');
disp('Successfully plot Xerror vesus time');
save('Xerr.mat','Xerr');


