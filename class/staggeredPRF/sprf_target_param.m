classdef sprf_target_param  < matlab.System
    properties
        position (3,1) double;
        rcs       = 2;
        velocity  = sprf_target_param.default_velocity;
        spin_radius = sprf_target_param.default_radius;
        spin_center (3,1) double = sprf_target_param.default_center;
        acceleration = sprf_target_param.default_acceleration;
    end
    properties(Constant)
        g = 9.8;
    end
    properties(Dependent)
        angular_speed
    end
    properties(Constant) %% defaults
        default_acceleration = 5*sprf_target_param.g;
        default_center = [700,700,300];
        default_radius = 500;
        default_velocity = [0;0;0];
    end

    methods
        function angular_speed = get.angular_speed(obj)
            angular_speed = sqrt(obj.acceleration / obj.spin_radius);
        end
        function obj= sprf_target_param(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
end

