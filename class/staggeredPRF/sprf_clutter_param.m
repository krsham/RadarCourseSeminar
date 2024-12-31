classdef sprf_clutter_param < matlab.System

    
    properties
        gamma   = 20;
        azimuthSpan = 360;
        patchAzimuthSpan = 30;
        numPulse = 10;
    end
    
    methods
        function obj = sprf_clutter_param(varargin)

            setProperties(obj,nargin,varargin{:});
        end

    end
end

