classdef sprf_scene_param < matlab.System

    
    properties
        beamWidth  = [5;5];
        beamRange  = 2000;
        updateRate = 75000;
        platformNames = {'Radar','Target'};
        trailLength      = 1e6;
    end
    
    methods
        function obj = sprf_scene_param(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end
end

