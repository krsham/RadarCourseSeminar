clc;clear;close all;
%% Operation
generate_filters        = true;
plot_filter_resp        = false;
motion_simulation       = false;
viz_mosim               = false;
propagate_receive       = false;
receiver_scope          = false;
no_clutter_processing   = true;
clutter_mode            = false;
%% Parameters
prf = 1e3;   % Radar PRF
Tstep  = 1/prf; % Time step of simulation
fc  = 10e9;             % carrier frequency
pd = 0.9;               % Probability of detection
pfa = 1e-6;             % Probability of false alarm
max_range = 2000;       % Maximum unambiguous range
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
txSpec.peakPower = 2e3;
clutterSpec.gamma   = surfacegamma('flatland');
clutterSpec.azimuthSpan = 360;
clutterSpec.patchAzimuthSpan = 10;
%% Viewer 
sceneSpec.beamWidth  = [5;5];
sceneSpec.beamRange  = 1250;
sceneSpec.updateRate = prf;
sceneSpec.platformNames = {'Radar','Target'};
sceneSpec.trailLength      = 1e6;
%% Kinematics
tgtSpec.init_angle       = 0;
target_acceleration = 5 * g; % m/s
angular_speed = sqrt(target_acceleration / spin_radius);
tgtSpec.vel  = spinningVelocity(angular_speed,spin_radius,tgtSpec.init_angle );
tgtSpec.pos      = spin_center + spin_radius*[cos(tgtSpec.init_angle ),sin(tgtSpec.init_angle ),0]';
tgtSpec.rcs   = 2;
%% Motion Sim
moSimFlush              = true;   % Flush output folder of previously generated data
moSimOutFolder          = 'generatedData/MotionSim'; % output folder for motion simulation
moSimOutTemplate        = [moSimOutFolder,'/motionSimulation'];
moSimIteration          = 10000;  % Save buffer after this iteration count
%% Propagation and basic reception
propReceiveFlush                = true;
propReceiveOutFolder          = 'generatedData/RXTX'; % output folder for motion simulation
propReceiveOutTemplate        = [propReceiveOutFolder,'/RXTXSignals'];
%% No Clutter Processing
noClutterFlush                = true;
noClutterOutFolder            = 'generatedData/noClutterProcess';
noClutterOutTemplate          = [noClutterOutFolder,'/noClutterProcessed']
%% Basic Calculations
c = physconst('LightSpeed');   % Propagation speed
pulse_bw = c/(2*range_res);    % Pulse bandwidth
pw = 1/pulse_bw;               % Pulse width
prf = c/(2*max_range);         % Pulse repetition frequency
fs = 10*pulse_bw;                        % Sampling rate
samplesPerWave = floor(fs/prf);
num_pulse_int = 10;            % Pulse Integrated
mf_res = max_range/samplesPerWave;
mf_ds_factor = range_res/mf_res;
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
    target,tgtmotion,channel,matchedfilter,tvg,threshold,clutter,scene] = ...
    helperDelayLineSpinningTargetSetup(fs,pw,prf,fc,c,txSpec,tgtSpec,clutterSpec,sceneSpec);
%% Motion Simulation
if motion_simulation
    buffSize = bufferSize(Nsteps,moSimIteration);
    motionSimBuffer.tgt_pos = zeros(3,buffSize);
    motionSimBuffer.tgt_ang = zeros(1,buffSize);
    motionSimBuffer.tgt_vel = zeros(3,buffSize);
    motionSimBuffer.rad_ang = zeros(2,buffSize);
    motionSimBuffer.rad_vel = zeros(3,buffSize);
    motionSimBuffer.index   = zeros(1,buffSize);
    if ~exist(moSimOutFolder,'dir')
        mkdir(moSimOutFolder);
    end
    if moSimFlush
        delete([moSimOutFolder,'/*'])
    end
    %
    init_tgt_vel = tgtSpec.vel;
    for mosimIndex = 1:Nsteps
        mosimBuffIndex = buffIndex(mosimIndex,moSimIteration);
        motionSimBuffer.index(mosimBuffIndex) = mosimIndex;
        if mosimIndex ==1 % Need to initialize target velocity to be used in current iteration
            [motionSimBuffer.tgt_pos(:,mosimBuffIndex)] = tgtmotion(Tstep,init_tgt_vel);
        elseif mosimBuffIndex == 1
            [motionSimBuffer.tgt_pos(:,mosimBuffIndex)] = tgtmotion(Tstep,motionSimBuffer.tgt_vel(:,moSimIteration));
        else
            [motionSimBuffer.tgt_pos(:,mosimBuffIndex)] = tgtmotion(Tstep,motionSimBuffer.tgt_vel(:,mosimBuffIndex-1));
        end
        motionSimBuffer.tgt_ang(:,mosimBuffIndex) = spinningAngle(spin_center,motionSimBuffer.tgt_pos(:,mosimBuffIndex));
        motionSimBuffer.tgt_vel(:,mosimBuffIndex) = spinningVelocity(angular_speed,spin_radius,motionSimBuffer.tgt_ang(:,mosimBuffIndex));
        [motionSimBuffer.rad_pos(:,mosimBuffIndex),motionSimBuffer.rad_vel(:,mosimBuffIndex)] = sensormotion(Tstep);
        [~,motionSimBuffer.rad_ang(:,mosimBuffIndex)] = rangeangle(motionSimBuffer.tgt_pos(:,mosimBuffIndex));
        if mosimBuffIndex == moSimIteration|| mosimIndex == Nsteps
            outFile = simNameGen(moSimOutTemplate);
            save(outFile,'motionSimBuffer');
        end
    end
    
    clear motionSimBuffer
end
%% Animate
if viz_mosim
    animoSimBuffSize  = bufferSize(Nsteps,moSimIteration);
    animosimSource = what(moSimOutFolder);
    animosimFiles = animosimSource.mat;
    % scene.hide
    scene.show
    tic;
    for animosimIndex = 1:Nsteps
        buffSize = bufferSize(Nsteps,moSimIteration);
        if mod(animosimIndex,1000) ==0
            animosimIndex
            toc
        end
        
        animosimBuffIndex = buffIndex(animosimIndex,moSimIteration);
        if animosimBuffIndex == 1 || animosimIndex == 1
            fileIndex = floor(animosimIndex/moSimIteration) + 1;
            inFile = [moSimOutFolder,'/',animosimFiles{fileIndex}];
            load(inFile,'motionSimBuffer');
        end
        scene.BeamSteering = motionSimBuffer.rad_ang(:,animosimBuffIndex);
        scene(motionSimBuffer.rad_pos(:,animosimBuffIndex),...
            motionSimBuffer.rad_vel(:,animosimBuffIndex),...
            motionSimBuffer.tgt_pos(:,animosimBuffIndex),...
            motionSimBuffer.tgt_vel(:,animosimBuffIndex));
        drawnow limitrate
        
    end
    clear animationScenes motionSimBuffer
end
%%
if propagate_receive
    buffSize = bufferSize(Nsteps,moSimIteration);
    propRecBuffer.pulse = zeros(samplesPerWave,buffSize);
    propRecBuffer.txpulse = zeros(samplesPerWave,buffSize);
    propRecBuffer.txstat = zeros(samplesPerWave,buffSize);
    propRecBuffer.radpulse = zeros(samplesPerWave,buffSize);
    propRecBuffer.chOut = zeros(samplesPerWave,buffSize);
    propRecBuffer.tgtReflPulse = zeros(samplesPerWave,buffSize);
    propRecBuffer.collSig = zeros(samplesPerWave,buffSize);
    propRecBuffer.rxpulse = zeros(samplesPerWave,buffSize);
    if ~exist(propReceiveOutFolder,'dir')
        mkdir(propReceiveOutFolder);
    end
    if propReceiveFlush
        delete([propReceiveOutFolder,'/*'])
    end
    propReceiveOutSource = what(moSimOutFolder);
    propReceiveOutFiles = propReceiveOutSource.mat;
    
    for rxtxIndex = 1:Nsteps
        noClutterBuffIndex = buffIndex(rxtxIndex,moSimIteration);
        if noClutterBuffIndex == 1 || rxtxIndex == 1
            fileIndex = floor(rxtxIndex/moSimIteration) + 1;
            inFile = [moSimOutFolder,'/',propReceiveOutFiles{fileIndex}];
            load(inFile,'motionSimBuffer');
        end
        propRecBuffer.pulse(:,noClutterBuffIndex) = waveform();
        [propRecBuffer.txpulse(:,noClutterBuffIndex),...
            propRecBuffer.txstat(:,noClutterBuffIndex)]= ...
            transmitter(propRecBuffer.pulse(:,noClutterBuffIndex));
        propRecBuffer.radpulse(:,noClutterBuffIndex) = ...
            radiator(propRecBuffer.txpulse(:,noClutterBuffIndex),motionSimBuffer.tgt_ang(:,noClutterBuffIndex));
        propRecBuffer.chOut(:,noClutterBuffIndex) = ...
            channel(propRecBuffer.radpulse(:,noClutterBuffIndex),motionSimBuffer.rad_pos(:,noClutterBuffIndex),...
            motionSimBuffer.tgt_pos(:,noClutterBuffIndex),motionSimBuffer.rad_vel(:,noClutterBuffIndex),...
            motionSimBuffer.rad_vel(:,noClutterBuffIndex));
        propRecBuffer.tgtReflPulse(:,noClutterBuffIndex) = ...
            target(propRecBuffer.chOut(:,noClutterBuffIndex));
        propRecBuffer.collSig(:,noClutterBuffIndex) = ... 
            collector(propRecBuffer.tgtReflPulse(:,noClutterBuffIndex),motionSimBuffer.tgt_ang(:,noClutterBuffIndex));
        propRecBuffer.rxpulse(:,noClutterBuffIndex) = ...
            receiver(propRecBuffer.collSig(:,noClutterBuffIndex),~(propRecBuffer.txstat(:,noClutterBuffIndex)>0));
        if noClutterBuffIndex == moSimIteration|| rxtxIndex == Nsteps
            outFile = simNameGen(propReceiveOutTemplate);
            save(outFile,'propRecBuffer');
        end
    end
end     

%%

if receiver_scope
    rx_scope = timescope('SampleRate',fs,'LayoutDimensions',[2,1],...
        'NumInputPorts',2,'MaximizeAxes','off','Name','Transmitter/Receiver', ...
        'TimeSpanSource','property'...
        );
    set(rx_scope,'ActiveDisplay',1,...
        'TimeSpan',num_pulse_int/prf,'Title','Transmitted Signal',...
        'YLabel','Magnitude','YLimits',[0 800],...
        'TimeSpanOverrunAction','Scroll');
    set(rx_scope,'ActiveDisplay',2,...
        'TimeSpan',num_pulse_int/prf,'Title','Received Signal',...
        'YLabel','Magnitude','YLimits',[0 3e-6],...
        'TimeSpanOverrunAction','Scroll');
    propRecBuffer =[];
    for rxtxScopeIndex = 1:Nsteps
        rxtxBuffIndex = buffIndex(rxtxScopeIndex,moSimIteration);
        propRecBuffer = feedLoad(propReceiveOutFolder,rxtxScopeIndex,...
            moSimIteration,rxtxBuffIndex,'propRecBuffer',propRecBuffer);
        rx_scope(propRecBuffer.txpulse(:,rxtxBuffIndex),...
            [repmat(threshold,size(propRecBuffer.collSig(:,rxtxBuffIndex)))...
            abs(propRecBuffer.rxpulse(:,rxtxBuffIndex))])
        pause(0.01)
    end

end
%% 


if no_clutter_processing
    noCluttIteration = ceil(moSimIteration/num_pulse_int);
    noCluttSteps = ceil(Nsteps/num_pulse_int);
    buffSize = bufferSize(Nsteps,noCluttIteration);

    initFolder(noClutterOutFolder,noClutterFlush);
    noClutterBufferOut.mf_pulse = zeros(samplesPerWave/mf_ds_factor,num_pulse_int,buffSize);
    noClutterBufferIn = [];
    mf_scope = timescope('SampleRate',fs,'LayoutDimensions',[2,1],...
        'NumInputPorts',2,'MaximizeAxes','off','Name','MF Output', ...
        'TimeSpanSource','property'...
        );
    set(mf_scope,'ActiveDisplay',1,...
        'TimeSpan',num_pulse_int/prf,'Title','MF Video',...
        'YLabel','Magnitude','YLimits',[0 24e-6],...
        'TimeSpanOverrunAction','Scroll');
    set(mf_scope,'ActiveDisplay',2,...
        'TimeSpan',1/prf,'Title','NCI Video',...
        'YLabel','Magnitude','YLimits',[0 2e-6]*num_pulse_int,...
        'TimeSpan',1/prf,'TimeSpanOverrunAction','Wrap');
    rti = phased.RTIScope('IQDataInput',false,'SampleRate',fs,...
        'TimeResolution',1/prf,"Name",'Range Time Scope','RangeResolution',range_res,...
        'TimeSpan',1);
    movAvg = dsp.MovingAverage(num_pulse_int,0);
    for noClutterIndex = 1:noCluttSteps
        if mod(noClutterIndex,1000) == 0
            noClutterIndex
        end
        noClutterBuffIndex = buffIndex(noClutterIndex,noCluttIteration);
        noClutterBufferIn = feedLoad(propReceiveOutFolder,noClutterIndex,...
            noCluttIteration,noClutterBuffIndex,'propRecBuffer',noClutterBufferIn);
        selectRange = (noClutterBuffIndex-1)*num_pulse_int+1:noClutterBuffIndex*num_pulse_int;
        pulseBuff = noClutterBufferIn.rxpulse(:,selectRange);
        % mf_pulse  = tvg(matchedfilter(pulseBuff))';
        mf_pulse = (downsample(matchedfilter(pulseBuff),mf_ds_factor))';
        integ_pulse = movAvg(abs(mf_pulse)); 
        % scope1_in  = abs(reshape(mf_pulse',[],1));
        % scope2_in  = [repmat(threshold,size(repelem(integ_pulse(:),num_pulse_int))),...
        %         repelem(integ_pulse(:),num_pulse_int)];
        % mf_scope(scope1_in...
        %     ,scope2_in);
        % rtiInput = abs(mf_pulse)';
        % rti(rtiInput);
        % drawnow 'limitrate'
        % pause(0.01)
        noClutterBufferOut.mf_pulse(:,:,noClutterBuffIndex) = mf_pulse';
        feedSave(noClutterOutTemplate,noClutterIndex,Nsteps,noClutterBufferOut,noClutterBuffIndex,noCluttIteration)
    end
end
%%



if clutter_mode

end