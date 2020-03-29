function Tf = calculateTf(T)
% Calculate T for each piece of the trajectory based on the Euclidean
% distance between start and end point

T_se_initial = evalin('base','T_se_initial');
T_standoff_initial = evalin('base','T_standoff_initial');
T_grasp = evalin('base','T_grasp');
T_standoff_final = evalin('base','T_standoff_final');% 10cm above
T_release = evalin('base','T_release');


d1 = norm(T_se_initial(1:3,4)-T_standoff_initial(1:3,4));
d2 = norm(T_standoff_initial(1:3,4)-T_grasp(1:3,4));
d5 = norm(T_standoff_initial(1:3,4)-T_standoff_final(1:3,4));
d6 = norm(T_standoff_final(1:3,4)-T_release(1:3,4));
d_total = d1 + d2*2 + d5 + d6*2;
t3 = 0.63;
t7 = 0.63;
t1 = d1*(T-t3-t7)/d_total;  
t2 = d2*(T-t3-t7)/d_total;
t4 = t2;
t5 = d5*(T-t3-t7)/d_total;
t6 = d6*(T-t3-t7)/d_total;
t8 = t6;


Tf = [t1,t2,t3,t4,t5,t6,t7,t8];
Tf = round(Tf*10^2)/10^2; %round to integer multiple of 0.01

end

