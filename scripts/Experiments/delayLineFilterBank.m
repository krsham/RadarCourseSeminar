clc;clear;close all;
%% Operation
generate_filters = true;
plot_filter_resp  = false;
%% Parameters
PRF = 1e3;   % Radar PRF
fs = PRF*1000; % Radar fs
Tstep  = 1/PRF; % Time step of simulation
fc  = 10e9;             % carrier frequency
pd = 0.9;               % Probability of detection
pfa = 1e-6;             % Probability of false alarm
max_range = 5000;       % Maximum unambiguous range
range_res = 50;         % Required range resolution
tgt_rcs = 1;            % Required target radar cross section
cancellers_order = 1:2:8;  % Delay Line Cancellers
sensorHeight      = 20;  % Height which the radar is stationed at
g = 9.8; % m/s
spin_center     = [1000,1000,300]'; % Center of target circle radius
spin_radius     = 100; % meters
simDuration     = 1 * 60; % Simulation duration
Nsteps = simDuration /Tstep; % Number of steps per simulation
txSpec.height = 20;
txSpec.peakPower = 1e3;
clutterSpec.gamma   = surfacegamma('flatland');
clutterSpec.azimuthSpan = 360;
clutterSpec.patchAzimuthSpan = 10;
%% Kinematics
tgtSpec.init_angle       = 0;
target_acceleration = 5 * g; % m/s
angular_speed = target_acceleration / (spin_radius^2);
tgtSpec.vel  = spinningVelocity(angular_speed,spin_radius,tgtSpec.init_angle );
tgtSpec.pos      = spin_center + spin_radius*[cos(tgtSpec.init_angle ),sin(tgtSpec.init_angle ),0]';
tgtSpec.rcs   = [2,2,2];

%% Basic Calculations
c = physconst('LightSpeed');   % Propagation speed
pulse_bw = c/(2*range_res);    % Pulse bandwidth
pw = 1/pulse_bw;               % Pulse width
prf = c/(2*max_range);         % Pulse repetition frequency
fs = 2*pulse_bw;                        % Sampling rate
%% delay line canceller Generation
if generate_filters
    % Generation
    filter_coeffs = cell(numel(cancellers_order),1);
    for i = 1:numel(cancellers_order)
        filter_coeffs{i} = cancellerCoeffGen(cancellers_order(i));
    end
end
%% plot response of filters
if plot_filter_resp
    % Plot response of all filters in a normalized manner
    % This allows to visualize the relative snr changes
    f = linspace(0,prf*9,2000); % freq Range
    fig1 = figure(1); hold on;
    for i = 1:numel(cancellers_order)
        h     = filter_coeffs{i};
        hresp = freqz(h,1,f,prf);
        plot(f/1000,20*log10(abs(hresp)));
    end
    ylim([-100,30])
    xlim([0,150])
    % Visualize the range out of 99% BW
    plot([f(1),f(end)],[-20,-20],'r--','LineWidth',2)
    hold off;
    grid on; xlabel('Doppler Frequency (kHz)'); ylabel('Magnitude (dB)');
    title('Frequency Response of Normalized N pulse Cancellers');
    legend_base = "-pulse canceller";
    legend_names = cell(numel(cancellers_order),1);
    for i = 1:numel(cancellers_order)
        legend_names{i}= strcat(string(cancellers_order(i)),legend_base);
    end
    legend(legend_names)
end
%% Setup

[waveform,transmitter,radiator,collector,receiver,sensormotion,...
    target,tgtmotion,channel,matchedfilter,tvg,threshold,clutter] = ...
    helperDelayLineSpinningTargetSetup(fs,pw,prf,fc,c,txSpec,tgtSpec,clutterSpec);
%% Motion Simulation

% for i = 1:Nsteps
% 
% end

