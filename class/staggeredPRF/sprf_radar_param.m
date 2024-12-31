classdef sprf_radar_param < matlab.System
    properties
        fc              = 10e9;
        prf             = sprf_radar_param.c/(2*2000); %% default range 2000
        pulse_bw        = sprf_radar_param.c/(2*50); %% default range resolution 50
        pw              = 1/sprf_radar_param.c/(2*50);
        peakPower       = 2e3;
        fs              = 10*1/sprf_radar_param.c/(2*50);
        num_pulse_int   = 10;
        position(3,1) double = [0,0,20]
                  
    end
    properties (Constant)
        g = 9.8
        c = 3e8;
    end
    properties (Dependent)
        max_range
        range_res
        samples_per_wave
        pri
        mf_res
    end
    

    methods
        function obj = sprf_radar_param(varargin)
            setProperties(obj,nargin,varargin{:});
        end
        function pri = get.pri(obj)
            pri = 1/obj.prf;
        end
        function max_range = get.max_range(obj)
            max_range = obj.c/(2*min(obj.prf));
        end
        
        function samples_per_wave = get.samples_per_wave(obj)
            samples_per_wave = floor(obj.fs/obj.prf);
        end
        function mf_res = get.mf_res(obj)
            mf_res = obj.max_range/obj.samples_per_wave;
        end
    end
end

