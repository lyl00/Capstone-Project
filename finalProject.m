%  *** This script include some useful code pieces *** % 
%  *** Feel free to copy any pieces for your implementation *** %

% Don't run this script!!! 

% root = fileparts(pwd);
% homework_name = "homework3";
% sourceCode_name = "mr";
% addpath(genpath(fullfile(root,homework_name,sourceCode_name)));
% mobile base parameters
% l = 0.47/2;
% w = 0.3/2;
% r = 0.0475;
% T_b0 = RpToTrans(eye(3),[0.1662,0,0.0026]');
% M_0e = RpToTrans(eye(3),[0.033,0,0.6546]');
% Blists = [[0,0,1,0,0.033,0]',[0,-1,0,-0.5076,0,0]',[0,-1,0,-0.3526,0,0]'...
%     [0,-1,0,-0.2176,0,0]',[0,0,1,0,0,0]'];
% cube configuration
% T_sc_initial = RpToTrans(eye(3),[1,0,0.025]');
% T_sc_goal = RpToTrans(rotz(-pi/2),[0,-1,0.025]');
% end-effector planned configuration 
% T_se_initial = [0,0,1,0;0,1,0,0;-1,0,0,0.5;0,0,0,1];
% T_standoff_initial = RpToTrans(roty(pi/2),[1,0,0.1]'); % 10cm above cube
% T_grasp = RpToTrans(roty(pi/2),[1,0,0.025]');
% T_standoff_final = RpToTrans(rotx(pi)*roty(-pi/2),[0,-1,0.1]');% 10cm above
% T_release = RpToTrans(rotx(pi)*roty(-pi/2),[0,-1,0.025]');
% Construct a cell array to 
% T_configure = {T_se_initial,T_standoff_initial,T_grasp,T_grasp,...
%     T_standoff_initial,T_standoff_final,T_release,T_release,...
%     T_standoff_final};
% 
% Create my youbot object
% Mybot = youbot(l,w,r,T_b0,M_0e,Blists);

% Milestone 2 test
% dt = 0.01;% manual choose
% T_total = 13;% manual choose
% Tf = calculateTf(T_total);%calculate weighted time for each piece
% Traj = [];
% grasp_state = 0;
% for i = 1:8
%     if i == 3 
%         grasp_state = 1;
%     elseif i == 7
%         grasp_state = 0;
%     end
%     
%     Trajectory = Mybot.TrajectoryGenerator(T_configure{i},T_configure{i+1},Tf(i),dt,grasp_state,'Screw',5);
%     Traj = [Traj;Trajectory];
% end
% writematrix(Traj,'Milestone2.csv');
% 
% Milestone 1 test
% dt = 0.01;
% q = [0,0.5,0.5];
% theta = [0,0,pi/2,pi/2,pi/2]';
% wheelAngle = [pi,pi,pi,pi];
% % constant speed command 
% u = [10,10,10,10];
% theta_dot = [pi/2,pi/2,pi/2,pi/2,pi/2];
% speed = [u,theta_dot];
% maxspeed = 12.3*ones(1,9);% take uniform max speed.
% % Initial property of object Mybot.
% Mybot.q = q;
% Mybot.wheelAngle = wheelAngle;
% Mybot.theta = theta;
% grasp_state = 0;
% % Loop for 100 times
% command_milestone1 = zeros(101,13);
% command_milestone1(1,:) = [q,theta',wheelAngle,grasp_state];
% for i = 2:101
%     config = [Mybot.q,Mybot.theta',Mybot.wheelAngle]; %12 elements
%     Mybot = Mybot.nextState(config,speed,dt,maxspeed);
%     command_milestone1(i,:) = [Mybot.q,Mybot.theta',Mybot.wheelAngle,grasp_state];
% end
% 
% writematrix(command_milestone1,'Milestone1.csv');




