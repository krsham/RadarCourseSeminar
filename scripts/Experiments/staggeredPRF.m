clc; clear all; close all;


radar_params = sprf_radar_param;
if true
    f = logspace(1,6,100000);
    r = radar_params.cancellers_response(f);
    plot(f,db(r)); grid on
    hold;
end
radar_params = radar_params.set_shrader_prf; 
if true
    r = radar_params.cancellers_response(f);
    plot(f,db(r));
    hold;
    title("Shrader Staggered PRF with canceller [1 -2 1]")
end
target_params = sprf_target_param;
scene_params = sprf_scene_param;
clutter_params = sprf_clutter_param;

experiment = sprf_experiment(radar_params,target_params,clutter_params,scene_params);
if false
    experiment.mosim();
end

if false % Not working! 101235150 would be the needed sample rate
    experiment.prop_simulation();
end


