classdef rf_radar_param < sprf_radar_param

    properties
        canceller_feedback_coeffs = [1 -1.56 0.67];
    end
    %%%
    methods
        function obj = set_notch(obj)
            obj.canceller_coeffs = [1,-2,1];
            obj.canceller_feedback_coeffs = [1 -1.56 0.67];
        end



        function resp = cancellers_response(obj,freq)
            for i = 1:numel(obj.prf)
                n = (1:numel(obj.canceller_coeffs))-1;
                exp_list = (exp(1j*2*pi/obj.prf(i)*freq(:))).^n;
                resp_prf(:,i) = sum(obj.canceller_coeffs.*exp_list,2);
            end
            resp = sum(abs(resp_prf).^2,2)/numel(obj.prf);
            
        end
        function [resp,wresp] = recursive_filter_response(obj,freq)
            for i = 1:numel(obj.prf)
                n = (1:numel(obj.canceller_coeffs))-1;
                exp_list = (exp(1j*2*pi/obj.prf(i)*freq(:))).^n;
                resp_prf_forward(:,i) = sum(obj.canceller_coeffs.*exp_list,2);
            end
            for i = 1:numel(obj.prf)
                n = (1:numel(obj.canceller_feedback_coeffs))-1;
                exp_list = (exp(1j*2*pi/obj.prf(i)*freq(:))).^n;
                resp_prf_feedback(:,i) = sum(obj.canceller_feedback_coeffs.*exp_list,2);
            end
            resp_total = resp_prf_forward./resp_prf_feedback;
            resp = sum(abs(resp_total).^2,2)/numel(obj.prf);
        end
    end
end

