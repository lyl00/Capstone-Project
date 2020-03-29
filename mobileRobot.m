% The 4-wheel mobile robot superclass (Mecanum ominidirectional wheels)
classdef mobileRobot
 
    properties
        l
        w
        Radius
        q % take the form [phi,x,y]
        wheelAngle
        q_dot
        vb
        u
    end
    
    properties (Constant)
        height = 0.0963
    end
    
    properties(Dependent)
        %MATLAB does not store a value for the dependent Area property. When you 
        %query the value of the Area property, MATLAB calls the get.function
        configuration       
        H_0
        H_phi
    end
    
    methods
        function obj = mobileRobot(half_length,half_width,wheel_radius)
            % Construct function for the mobile base
            obj.l = half_length;
            obj.w = half_width;
            obj.Radius = wheel_radius;
        end
        
        function obj = set.u(obj,val)
             obj.u = val;
        end
  
        function val = get.configuration(obj)
           R = rotz(obj.q(1));
           p = [obj.q(2),obj.q(3),obj.height]';
           val = RpToTrans(R,p);
        end
        
%         function val = get.u(obj)
%            val = obj.H_0*obj.vb;
%         end
        
        function val = get.H_0(obj)
           val = (1/obj.Radius)*[-obj.l-obj.w,1,-1;obj.l+obj.w,1,1;...
             obj.l+obj.w,1,-1;-obj.l-obj.w,1,1];
        end
        
        function val = get.H_phi(obj)
           val = (1/obj.Radius)*[-obj.l-obj.w,1,-1;obj.l+obj.w,1,1;...
             obj.l+obj.w,1,-1;-obj.l-obj.w,1,1]*[1,0,0;0,cos(obj.q(3))...
             ,sin(obj.q(3));0,-sin(obj.q(3)),cos(obj.q(3))];
        end
       function obj = nextState(obj,u,dt,maxSpeed)
            % Using odometry to update current configuration q of the mobile base
            % subclass "youbot" could do whole body kinematic by
            % overwrite this function
            % If using this function iteratively, just use take obj.q as q 
            
            % First give a current q, my class will get current
            % configuration automatically
            % using logical matrix to index
            %  obj.q = q;
            u(find((u>=maxSpeed) == 1)) = 12.3;
            
            % Do odometry
            obj.vb = pinv(obj.H_0) * u';
            vb6 = [0,0,obj.vb(1),obj.vb(2),obj.vb(3),0]';
            T_bk_bknext = MatrixExp6(VecTose3(vb6*dt));
            T_s_bknext = obj.configuration*T_bk_bknext;

            % extract new q from T_s_bknext which can be the input in next
            % step. update the mobile base configurtaion
            obj.q = [atan2(T_s_bknext(2,1),T_s_bknext(1,1)),T_s_bknext(1,4),T_s_bknext(2,4)];
            obj.wheelAngle = obj.wheelAngle + u*dt;
            
       end
    end
end

