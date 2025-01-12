classdef sprf_radar_param < matlab.System
    properties
        fc              = 10e9;
        prf             = sprf_radar_param.c/(2*2000); %% default range 2000
        pulse_bw        = sprf_radar_param.c/(2*50); %% default range resolution 50
        pw              = 1/sprf_radar_param.c/(2*50);
        peakPower       = 2e3;
        fs              = 10*1*sprf_radar_param.c/(2*50);
        num_pulse_int   = 10;
        position(3,1) double = [0,0,20]
        canceller_coeffs = [1 -2 1];
                  
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
        first_blind_speed
        mf_ds_factor
        fs_integrated
    end
    

    methods
        function fs_integrated = get.fs_integrated(obj)
            fs_integrated= obj.fs/obj.num_pulse_int/obj.mf_ds_factor;
        end
        function range_res = get.range_res(obj)
            range_res = 50;
        end
        function mf_ds_factor = get.mf_ds_factor(obj)
            mf_ds_factor = obj.range_res/obj.mf_res;
        end
        function fps = get.first_blind_speed(obj)
            lambda = obj.c/obj.fc;
            fps = lambda * min(obj.prf) / 2;
        end
        function obj = set_shrader_prf(obj)
            f_b = round(min(obj.prf)/31);
            obj.prf = f_b*[25,30,27,31];
        end

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
            samples_per_wave = floor(obj.fs/max(obj.prf));
        end
        function mf_res = get.mf_res(obj) %% matched filter resolution
            mf_res = obj.max_range/obj.samples_per_wave;
        end
        function resp = cancellers_response(obj,freq)
            for i = 1:numel(obj.prf)
                n = (1:numel(obj.canceller_coeffs))-1;
                exp_list = (exp(1j*2*pi/obj.prf(i)*freq(:))).^n;
                resp_prf(:,i) = sum(obj.canceller_coeffs.*exp_list,2);
            end
            resp = sum(abs(resp_prf).^2,2)/numel(obj.prf);
        end
        
    end
end

