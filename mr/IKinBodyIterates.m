function [thetalist, success, theta_iterates] = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev, log_name)
% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Blist: The joint screw axes in the end-effector frame when the
%              manipulator is at the home position, in the format of a 
%              matrix with the screw axes as the columns,
%       M: The home configuration of the end-effector,
%       T: The desired end-effector configuration Tsd,
%       thetalist0: An initial guess of joint angles that are close to 
%                   satisfying Tsd,
%       eomg: A small positive tolerance on the end-effector orientation
%             error. The returned joint angles must give an end-effector 
%             orientation error less than eomg,
%       ev: A small positive tolerance on the end-effector linear position 
%           error. The returned joint angles must give an end-effector
%           position error less than ev.
%       log_name: specify the .txt name of the recording file, by default
%                 is "log.txt". 
% Returns thetalist: Joint angles that achieve T within the specified 
%                    tolerances,
%         success: A logical value where TRUE means that the function found
%                  a solution and FALSE means that it ran through the set 
%                  number of maximum iterations without finding a solution
%                  within the tolerances eomg and ev.
%         theta_iterates: record the calculated joint angle for each
%                   iteration.
%           
% Uses an iterative Newton-Raphson root-finding method.
% The maximum number of iterations before the algorithm is terminated has 
% been hardcoded in as a variable called maxiterations. It is set to 20 at 
% the start of the function, but can be changed if needed.  
% Example Inputs:
% 
% clear; clc;
% Blist = [[0; 0; -1; 2; 0; 0], [0; 0; 0; 0; 1; 0], [0; 0; 1; 0; 0; 0.1]];
% M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
% T = [[0, 1, 0, -5]; [1, 0, 0, 4]; [0, 0, -1, 1.6858]; [0, 0, 0, 1]];
% thetalist0 = [1.5; 2.5; 3];
% eomg = 0.01;
% ev = 0.001;
% [thetalist, success] = IKinBody(Blist, M, T, thetalist0, eomg, ev)
% 
% Output:
% thetalist =
%    1.5707
%    2.9997
%    3.1415
% success =
%     1
if nargin == 6
    str = 'log.txt';
else 
    str = log_name;
end

fileID = fopen(str, 'w');
thetalist = thetalist0;
n = size(thetalist,1);
i = 0;
maxiterations = 20;
theta_iterates = zeros(maxiterations + 1,n);
theta_iterates(1,:) = thetalist';

Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev; % a logical value

while err && i < maxiterations
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
    theta_iterates(i+2,:) = thetalist';
    i = i + 1;
    Tsb = FKinBody(M,Blist,thetalist);
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    
    fprintf(fileID,'%s %d\n\n','Iteration:',i);
    fprintf(fileID,'%12s\n','joint vector:');
    for j = 1:n
        if j < n
            fprintf(fileID,'%0.3f, ',thetalist(j));
        else
            fprintf(fileID,'%0.3f\n',thetalist(j));
        end
    end
    fprintf(fileID,'%s\n','SE(3) end-effector config:');
    fprintf(fileID,'%.3f %.3f %.3f %.3f\n',Tsb');
    fprintf(fileID,'%s\n','error twist V_b:');
    fprintf(fileID,'%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n',Vb);
    fprintf(fileID,'%s: %.3f\n','angular error magnitude ||omege_b||:',norm(Vb(1:3)));
    fprintf(fileID,'%s: %.4f\n\n','linear error magnitude ||v_b||:',norm(Vb(4:6)));     
end
theta_iterates = theta_iterates(1:i+1,:); 
fclose(fileID);
writematrix(theta_iterates,'iterates.csv');

success = ~ err;
end