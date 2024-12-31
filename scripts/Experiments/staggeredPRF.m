clc; clear all; close all;


radar_params = sprf_radar_param;
target_params = sprf_target_param;
scene_params = sprf_scene_param;
clutter_params = sprf_clutter_param;

experiment = sprf_experiment(radar_params,target_params,clutter_params,scene_params);
experiment.mosim()

