clc; clear all; close all;


radar_params = rf_radar_param;
if true
    f = logspace(1,5.6,10000);
    [r] = radar_params.cancellers_response(f);
    plot(f,db(r)); grid on
    hold; title("canceller respone vs recursive filter (Chebyshev order 2)")

    radar_params = radar_params.set_notch;
    [r] = radar_params.recursive_filter_response(f);
    plot(f,db(r));
end

target_params = rf_target_param;
scene_params = rf_scene_param;
clutter_params = rf_clutter_param;

experiment = rf_experiment(radar_params,target_params,clutter_params,scene_params);
if false
    experiment = experiment.mti_process();
end

if false
    experiment = experiment.mti_demo();
end

