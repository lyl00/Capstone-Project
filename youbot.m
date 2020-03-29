% Subclass youbot, with a 5R robot arm articulated on the mobile base
classdef youbot < mobileRobot

    properties
        theta
        theta_dot
        M_be
        Blists
        T_be
        T_se
        Trajectory 
        kp
        ki
    end
    properties(Dependent)
        Je
    end
    methods
        function obj = youbot(half_length,half_width,wheel_radius,T_b0,M_0e,Blists,thetalist)
            % Construct the youbot AT its zero position
            % using robot arm configuration
            obj@mobileRobot(half_length,half_width,wheel_radius);
            obj.Blists = Blists;
            if nargin < 7
                obj.M_be = T_b0*M_0e;
            else 
                obj.theta = thetalist;
                obj.M_be = T_b0*M_0e;
                obj.T_be = FKinBody(obj.M_be,Blists,thetalist);
            end
        end
        
        function val = get.Je(obj)
            J_arm = JacobianBody(obj.Blists,obj.theta);
            F = pinv(obj.H_0,0.0001);
            F6 = [0,0,0,0;0,0,0,0;F;0,0,0,0];
            J_base = Adjoint(TransInv(obj.T_be))*F6;
            val = [J_base,J_arm];
        end

        
        function obj = forwardkinematic(obj)
            val = FKinBody(obj.M_be,obj.Blists,obj.theta);
            obj.T_be = val;
            obj.T_se = obj.configuration * val;
        end
        
        function Traj = cell2traj(obj,grasp_flag)
            %Trajectory is a cell array constructed of one piece trajectory 
            num_frame = numel(obj.Trajectory);
            Traj = zeros(num_frame,13);
            for i = 1:num_frame
                Rp = obj.Trajectory{i}(1:3,1:4);
                R = transpose(Rp(1:3,1:3));
                Traj(i,:) = [reshape(R,1,[]),transpose(Rp(1:3,4)),grasp_flag];
            end
        end
        
        function Traj = TrajectoryGenerator(obj,Tstart,Tend,T,dt,...
                graspe_flag,method,scaling_order)
            N = round(T/dt + 1);
            % default order is 3
            if nargin < 7
                if strcmp(method,'Screw')
                    obj.Trajectory = ScrewTrajectory(Tstart,Tend,T,N,3);
                elseif strcmp(method,'Cartesian')
                    obj.Trajectory = CartesianTrajectory(Tstart,Tend,T,N,3);
                end
                
            else
                 if strcmp(method,'Screw')
                    obj.Trajectory = ScrewTrajectory(Tstart,Tend,T,N,...
                        scaling_order);
                elseif strcmp(method,'Cartesian')
                    obj.Trajectory = CartesianTrajectory(Tstart,Tend,T,N,...
                        scaling_order);
                end
                
            end
            % This function extract from the configuration cell array to
            % form the needed trajectory parameter:[r11,r12......] 
            Traj = cell2traj(obj,graspe_flag); 
            
            
        end
       function obj = nextState(obj,config,speed,dt,maxSpeed)
            % Using odometry to get current configuration of the mobile base
            % subclass "youbot" could do whole body kinematic by
            % overwrite the function in the superclass
            % Input: maxSpeed is 9 element
            %        config is 12 element
            %        speed is 9 element
            % First invoke same method in superclass
            % Here must be an obj return!!!!!A great mistake, or q,wheelAngle 
            % will never be updated!
            obj = nextState@mobileRobot(obj,speed(1:4),dt,maxSpeed(1:4));
            % using logical matrix to index which element the speed is
            % bigger 
            % than maxSpeed
            speed = speed(5:9);
            maxSpeed = maxSpeed(5:9);
            speed(find((speed>=maxSpeed)==1)) = 12.3;
            
            obj.theta = config(4:8) + speed*dt; %update arm configuration
            obj.theta = obj.theta';% should be column vector as a property
       end
       function [Animation,Xerr,obj] = FeedbackControl(obj,dt,Td,...
               maxspeed,grasp,jointLimits)
           N = size(Td,3);
           Animation = zeros(N,13);
           Xerr = zeros(6,N-1);
           V = zeros(6,N-1);
           Animation(1,:) = [obj.q,obj.theta',obj.wheelAngle,grasp(1)];
           for i = 1:N-1
                obj = obj.forwardkinematic;
                Tse = obj.T_se; % current configuration of the end-effector
                obj.T_be = FKinBody(obj.M_be,obj.Blists,obj.theta);
                Xd = (1/dt)*se3ToVec(MatrixLog6(TransInv(Td(:,:,i))*...
                    Td(:,:,i+1)));
                xerr = se3ToVec(MatrixLog6(TransInv(Tse)*Td(:,:,i)));
                Xerr(:,i) = xerr;
                Xerr_sum = sum(Xerr,2);% The integral of error twist
                % Velocity input control(Feedforward + PI fb)
                V(:,i) = Adjoint(TransInv(Tse)*Td(:,:,i))*Xd...
                    + obj.kp * xerr + obj.ki * Xerr_sum * dt;
                speed = pinv(obj.Je,0.0090) * V(:,i);%speed = [u,theta_dot]'
                obj.u = speed(1:4);
                obj.theta_dot = speed(5:9);
                speed = [obj.u',obj.theta_dot'];
                config = [obj.q,obj.theta',obj.wheelAngle];
                % Do a joint limit test to avoind collision,then calculate
                % speed again
                Je_new = testJointLimits(obj,jointLimits,speed,config,dt,...
                    maxspeed);
                speed = pinv(Je_new,0.0090) * V(:,i);%speed = [u,theta_dot]'
                obj.u = speed(1:4);
                obj.theta_dot = speed(5:9);
                speed = [obj.u',obj.theta_dot'];
                % use the kinematic simulator to update the obj's properties
                obj = obj.nextState(config,speed,dt,maxspeed); 
                Animation(i+1,:) = [obj.q,obj.theta',obj.wheelAngle,...
                    grasp(i)];
           end
       end
       function Je = testJointLimits(obj,jointLimits,speed,config,dt,...
               maxspeed)
           obj_test = obj.nextState(config,speed,dt,maxspeed);
           num_joint = size(jointLimits,2);
           Je = obj.Je;
           theta_test = obj_test.theta';
           for i = 1:num_joint
               if theta_test(i) > jointLimits(1,i)||theta_test(i) < jointLimits(2,i)
                   Je(:,i+4) = zeros(6,1);
               end
                   
           end
       end
       
    end
    
    
    
end

