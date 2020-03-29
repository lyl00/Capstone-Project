function [Td,grasp] = traj2mat(Traj)
% This function extract element from the Trajectory to generate the 
% target configuration and the grasp state.
% ie:[r11,r12,r13,r21,r22,r23,r31,r32,r33,px, py, pz,grasp_state] ->
% Td and grasp_state
% The input Traj is a N*13 matrix
N = size(Traj,1);
Td = zeros(4,4,N);
grasp = zeros(1,N);

for i =1 : N
    Rd = reshape(Traj(i,1:9),[3,3]); Rd = Rd';
    Pd = Traj(i,10:12); Pd = Pd';
    Tdi = RpToTrans(Rd,Pd);
    Td(:,:,i) = Tdi;
    grasp(i) = Traj(i,13);
end

end

