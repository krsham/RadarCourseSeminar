classdef sprf_sim_param < matlab.System
    %SPRF_SIM_PARAM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        logFolderRoot = 'generatedData/staggeredPRF'
        Tstep         = 
    end
    
    methods
        function obj = sprf_sim_param(varargin)
            setProperties(obj,nargin,varargin{:});
        end
        
    end
end

