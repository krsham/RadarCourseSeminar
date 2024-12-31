clc;clear; close all;
%% Operation
generate_filters        = true;
plot_filter_resp        = false;
motion_simulation       = false;
viz_mosim               = false;
propagate_receive       = false;
receiver_scope          = false;
no_clutter_processing   = false;
no_clutter_scope        = false;
generate_clutter        = false;
clutter_mode            = false;
clutter_demo            = false;
mti_process             = false;
mti_demo                = true;
%% Parameters
fc  = 10e9;             % carrier frequency
pd = 0.9;               % Probability of detection
pfa = 1e-6;             % Probability of false alarm
max_range = 2000;       % Maximum unambiguous range
range_res = 50;         % Required range resolution
tgt_rcs = 1;            % Required target radar cross section
cancellers_order = 1:2:8;  % Delay Line Cancellers
g = 9.8; % m/s
spin_center     = [700,700,300]'; % Center of target circle radius
spin_radius     = 500; % meters
simDuration     = 30; % Simulation duration
txSpec.height = 20;
txSpec.peakPower = 2e3;
% clutterSpec.gamma   = surfacegamma('flatland');
clutterSpec.gamma   = 20;
clutterSpec.azimuthSpan = 360;
clutterSpec.patchAzimuthSpan = 30;
visualization_ds_fac = 1;
%% Basic Calculations
c = 3e8;   % Propagation speed
pulse_bw = c/(2*range_res);    % Pulse bandwidth
pw = 1/pulse_bw;               % Pulse width
prf = c/(2*max_range);         % Pulse repetition frequency
fs = 10*pulse_bw;                        % Sampling rate
samplesPerWave = floor(fs/prf);
num_pulse_int = 10;            % Pulse Integrated
clutterSpec.numPulse         = num_pulse_int;
mf_res = max_range/samplesPerWave;
mf_ds_factor = range_res/mf_res;
Tstep  = 10/prf; % Time step of simulation
Nsteps = simDuration /Tstep; % Number of steps per simulation
%% Viewer
sceneSpec.beamWidth  = [5;5];
sceneSpec.beamRange  = 2000;
sceneSpec.updateRate = prf/visualization_ds_fac;
sceneSpec.platformNames = {'Radar','Target'};
sceneSpec.trailLength      = 1e6;
%% Kinematics
tgtSpec.init_angle       = 0;
target_acceleration = 5 * g; % m/s
angular_speed = sqrt(target_acceleration / spin_radius);
tgtSpec.vel   = spinningVelocity(angular_speed,spin_radius,tgtSpec.init_angle );
tgtSpec.pos      = spin_center + spin_radius*[cos(tgtSpec.init_angle ),sin(tgtSpec.init_angle ),0]';
tgtSpec.rcs   = 2;
%% Foldering
rootFolderName  = 'generatedData';
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
noClutterReceptionFlush                = true;
noClutterReceptionOutFolder            = 'generatedData/noClutterProcess';
noClutterReceptionOutTemplate          = [noClutterReceptionOutFolder,'/noClutterProcessed'];
%% Clutter Out Folder

clutterReceptionOutFolder              = 'generatedData/clutterProcess';
clutterReceptionOutTemplate            = [clutterReceptionOutFolder,'/clutterOutProcessed'];
%% Clutter Reception
clutterFlush                            = true;
clutterFolder                           = 'generatedData/clutterData';
clutterTemplate                         = [clutterFolder,'/clutterGenerated'];
%% MTI Processing
mtiFlush                                = true;
[mtiFolder,mtiTemplate]                 = saveFolderDef(rootFolderName,'mtiData','mtiProcessed');



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
    for animosimIndex = 1:visualization_ds_fac:Nsteps
        buffSize = bufferSize(Nsteps,moSimIteration);
        if mod(animosimIndex,1000) == 1
            animosimIndex
            toc
        end
        
        animosimBuffIndex = buffIndex(animosimIndex,moSimIteration);
        if animosimBuffIndex == 1 || animosimIndex == 1
            fileIndex = floor(animosimIndex/moSimIteration) + 1;
            inFile = [moSimOutFolder,'/',animosimFiles{fileIndex}];
            load(inFile,'motionSimBuffer');abs(mf_pulse)
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
%% Propagation Simulation
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
    initFolder(propReceiveOutFolder,propReceiveFlush)
    motionSimBuffer = [];
    for rxtxIndex = 1:Nsteps
        if mod(rxtxIndex,1000) == 1
            rxtxIndex
            toc
        end
        noClutterReceptionBuffIndex = buffIndex(rxtxIndex,moSimIteration);
        motionSimBuffer = feedLoad(moSimOutFolder,rxtxIndex,noClutterReceptionBuffIndex,moSimIteration,'motionSimBuffer',motionSimBuffer);
        
        tgt_ang = motionSimBuffer.tgt_ang(:,noClutterReceptionBuffIndex);
        tgt_pos = motionSimBuffer.tgt_pos(:,noClutterReceptionBuffIndex);
        rad_pos = motionSimBuffer.rad_pos(:,noClutterReceptionBuffIndex);
        rad_vel = motionSimBuffer.rad_vel(:,noClutterReceptionBuffIndex);
        tgt_vel = motionSimBuffer.tgt_vel(:,noClutterReceptionBuffIndex);
        
        
        ppulse = waveform();
        [ptxpulse,txStat] = transmitter(ppulse);
        radPulse = radiator(ptxpulse,tgt_ang);
        chOut = channel(radPulse,rad_pos,tgt_pos,rad_vel,tgt_vel);
        tgtRefpulse = target(chOut);
        collSig = collector(tgtRefpulse,tgt_ang);
        prxPulse = receiver(collSig,~(txStat>0));
        %%%
        propRecBuffer.pulse(:,noClutterReceptionBuffIndex)        = ppulse;
        propRecBuffer.txpulse(:,noClutterReceptionBuffIndex)      = ptxpulse;
        propRecBuffer.tgtReflPulse(:,noClutterReceptionBuffIndex) = tgtRefpulse;
        propRecBuffer.collSig(:,noClutterReceptionBuffIndex)      = collSig;
        propRecBuffer.txstat(:,noClutterReceptionBuffIndex)       = txStat;
        propRecBuffer.radpulse(:,noClutterReceptionBuffIndex)     = radPulse;
        propRecBuffer.chOut(:,noClutterReceptionBuffIndex)        = chOut;
        propRecBuffer.rxpulse(:,noClutterReceptionBuffIndex)      = prxPulse;
        %%%
        if noClutterReceptionBuffIndex == moSimIteration|| rxtxIndex == Nsteps
            outFile = simNameGen(propReceiveOutTemplate);
            save(outFile,'propRecBuffer');
        end
    end
end

%% Receiver Scope View

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
    for rxtxScopeIndex = 1:visualization_ds_fac:Nsteps
        rxtxBuffIndex = buffIndex(rxtxScopeIndex,moSimIteration);
        propRecBuffer = feedLoad(propReceiveOutFolder,rxtxScopeIndex,...
            moSimIteration,rxtxBuffIndex,'propRecBuffer',propRecBuffer);
        rx_scope(propRecBuffer.txpulse(:,rxtxBuffIndex),...
            [repmat(threshold,size(propRecBuffer.collSig(:,rxtxBuffIndex)))...
            abs(propRecBuffer.rxpulse(:,rxtxBuffIndex))])
        pause(0.01)
    end
    
end
%% No Clutter Range detection

% Integrated info
integratedIteration = ceil(moSimIteration/num_pulse_int);
integratedSteps = ceil(Nsteps/num_pulse_int);
buffSize = bufferSize(Nsteps,integratedIteration);

if no_clutter_processing
    
    initFolder(noClutterReceptionOutFolder,noClutterReceptionFlush);
    noClutterReceptionBufferOut.mf_pulse = zeros(samplesPerWave/mf_ds_factor,num_pulse_int,buffSize);
    noClutterReceptionBufferOut.integ_pulse = zeros(samplesPerWave/mf_ds_factor,buffSize);
    noClutterReceptionBufferIn = [];
    
    movAvg = dsp.MovingAverage(num_pulse_int,0);
    for noClutterReceptionIndex = 1:integratedSteps
        if mod(noClutterReceptionIndex,integratedIteration) == 0
            noClutterReceptionIndex
        end
        noClutterReceptionBuffIndex = buffIndex(noClutterReceptionIndex,integratedIteration);
        noClutterReceptionBufferIn = feedLoad(propReceiveOutFolder,noClutterReceptionIndex,...
            noClutterReceptionBuffIndex,buffSize,'propRecBuffer',noClutterReceptionBufferIn);
        selectRange = (noClutterReceptionBuffIndex-1)*num_pulse_int+1:noClutterReceptionBuffIndex*num_pulse_int;
        pulseBuff = noClutterReceptionBufferIn.rxpulse(:,selectRange);
        % mf_pulse  = tvg(matchedfilter(pulseBuff))';
        mf_pulse = (downsample(matchedfilter(pulseBuff),mf_ds_factor))';
        integ_pulse = movAvg(abs(mf_pulse));
        noClutterReceptionBufferOut.mf_pulse(:,:,noClutterReceptionBuffIndex) = mf_pulse';
        noClutterReceptionBufferOut.integ_pulse(:,noClutterReceptionBuffIndex) = integ_pulse';
        feedSave(noClutterReceptionOutTemplate,noClutterReceptionIndex,integratedSteps,noClutterReceptionBufferOut,noClutterReceptionBuffIndex,integratedIteration)
    end
end
%% No Clutter Scope

if no_clutter_scope
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
    noClutterReceptionBufferOut =[];
    for noClutterReceptionIndex = 1:visualization_ds_fac:integratedSteps
        noClutterReceptionBuffIndex = buffIndex(noClutterReceptionIndex,integratedIteration);
        noClutterReceptionBufferOut = feedLoad(noClutterReceptionOutFolder,noClutterReceptionIndex,...
            noClutterReceptionBuffIndex,buffSize,'noClutterReceptionBufferOut',noClutterReceptionBufferOut);
        
        mf_pulse = noClutterReceptionBufferOut.mf_pulse(:,:,noClutterReceptionBuffIndex);
        integ_pulse = noClutterReceptionBufferOut.integ_pulse(:,noClutterReceptionBuffIndex);
        scope1_in  = abs(reshape(mf_pulse',[],1));
        scope2_in  = [repmat(threshold,size(repelem(integ_pulse(:),num_pulse_int))),...
            repelem(integ_pulse(:),num_pulse_int)];
        mf_scope(scope1_in...
            ,scope2_in);
        rtiInput = integ_pulse;
        rti(rtiInput);
        drawnow 'limitrate'
        pause(0.01)
    end
    
end



%% Clutter No MTI


clutterBufferIn = [];

if generate_clutter
    initFolder(clutterFolder,true);
    clutterGenerated  =  zeros(400,integratedIteration);
    for i = 1:integratedIteration
        if mod(i,100) == 0
            i
        end
        x = clutter();
        s = clutterSpec.numPulse;
        clutterGenerated(:,(i-1)*s+1:(i)*s) = reshape(x,400,clutterSpec.numPulse);
    end
    n = simNameGen(clutterTemplate);
    save(n,'clutterGenerated');
end

if clutter_mode
    initFolder(clutterReceptionOutFolder,clutterFlush);
    movAvg = dsp.MovingAverage(num_pulse_int,0);
    
    clutterRxBuff.mf_pulse_cluttered = zeros(samplesPerWave,moSimIteration);
    
    clutterData      = feedLoad(clutterFolder,1);
    for clutterStep = 1:integratedSteps
        if mod(clutterStep,1000) == 0
            clutterStep
        end
        clutterBuffIndex = buffIndex(clutterStep,integratedIteration);
        clutterBufferIn = feedLoad(propReceiveOutFolder,clutterStep,...
            clutterBuffIndex,buffSize,'propRecBuffer',clutterBufferIn);
        selectRange = batchSelection(num_pulse_int,clutterBuffIndex);
        pulseBuff = clutterBufferIn.rxpulse(:,selectRange);
        clutterBuff = clutterData.clutterGenerated(:,selectRange);
        clutterPulse = clutterBuff + pulseBuff;
        mf_pulse = matchedfilter((clutterPulse));
        mf_pulse_ds = (downsample(mf_pulse,mf_ds_factor))';
        integ_pulse = movAvg(abs(mf_pulse_ds));
        clutterRxBuff.mf_pulse_cluttered(:,selectRange) = mf_pulse;
        feedSave(clutterReceptionOutTemplate,clutterStep,integratedSteps,clutterRxBuff,clutterBuffIndex,buffSize);
    end
    clear clutterRxBuff clutterBufferIn
    
end
%% Clutter demo
if clutter_demo
    clear mf_scope rti
    mf_scope = timescope('SampleRate',fs,'LayoutDimensions',[1,1],...
        'NumInputPorts',1,'MaximizeAxes','off','Name','MF Output', ...
        'TimeSpanSource','property'...
        );
    set(mf_scope,'ActiveDisplay',1,...
        'TimeSpan',num_pulse_int/prf,'Title','MF Video',...
        'YLabel','Magnitude','YLimits',[0 35e-3],...
        'TimeSpanOverrunAction','Scroll');
    rti = phased.RTIScope('IQDataInput',false,'SampleRate',fs,...
        'TimeResolution',1/prf,"Name",'Range Time Scope','RangeResolution',range_res,...
        'TimeSpan',0.2);
    
    clutterBufferIn = [];
    for clutterStep = 1:moSimIteration
        if mod(clutterStep,1000) == 0
            clutterStep
        end
        clutterDemoBuffIndex     = buffIndex(clutterStep,moSimIteration);
        clutterBufferIn      = feedLoad(clutterReceptionOutFolder,clutterStep,clutterDemoBuffIndex...
            ,moSimIteration,'clutterRxBuff',clutterBufferIn);
        mf_pulse = clutterBufferIn.mf_pulse_cluttered(:,clutterDemoBuffIndex);
        scope_in  = abs(mf_pulse);
        mf_scope(scope_in);
        rtiInput = scope_in;
        rti(rtiInput);
        drawnow limitrate
        pause(0.01)
    end
    
end

%% MTI Mode

if mti_process
    initFolder(mtiFolder,mtiFlush);
    mtiBuffer.mfMtiFiltered         = zeros(samplesPerWave,moSimIteration,numel(filter_coeffs));
    filterStates                    = cell(numel(filter_coeffs),1);
    mtiBuffer.integratedMtiFiltered = zeros(samplesPerWave/num_pulse_int,integratedIteration,numel(filter_coeffs));
    movAvg = dsp.MovingAverage(num_pulse_int,0);
    mtiBufferIn = [];
    
    
    for mtiStep = 1:integratedSteps
        if mod(mtiStep,1000) == 0
            mtiStep
        end
        mtiBuffIndex     = buffIndex(mtiStep,integratedIteration);
        mtiBufferIn  = feedLoad(clutterReceptionOutFolder,mtiStep,mtiBuffIndex...
            ,integratedIteration,'clutterRxBuff',mtiBufferIn);
        selectRange = batchSelection(num_pulse_int,mtiBuffIndex);
        cluttered_in      = mtiBufferIn.mf_pulse_cluttered(:,selectRange);
        for filterIndex = 1:numel(filter_coeffs)
            currentFilter                   = filter_coeffs{filterIndex}; currentState = filterStates{filterIndex};
            [filtOut,currentState]  = filter(currentFilter,1,cluttered_in,currentState,2);mtiFilteredData = filtOut;
            filterStates{filterIndex}       = currentState;
            mtiBuffer.mfMtiFiltered(:,selectRange,filterIndex)  =  mtiFilteredData ;
            mtiFilteredDs(:,:,filterIndex) = downsample(mtiFilteredData,mf_ds_factor);
            integ_mti(:,filterIndex)     =  movAvg(abs(mtiFilteredDs(:,:,filterIndex) )')';
            mtiBuffer.integratedMtiFiltered(:,mtiBuffIndex,filterIndex)  =  integ_mti(:,filterIndex) ;
        end
        feedSave(mtiTemplate,mtiStep,integratedSteps,mtiBuffer,mtiBuffIndex,buffSize);
    end
    
end
clear rti mf_scope
fs_integrated =  fs/num_pulse_int/mf_ds_factor;
if mti_demo
    
    mf_scope = timescope('SampleRate',fs_integrated,'LayoutDimensions',[1,1],...
        'NumInputPorts',1,'MaximizeAxes','off','Name','MF Output', ...
        'TimeSpanSource','property'...
        );
    set(mf_scope,'ActiveDisplay',1,...
        'TimeSpan',num_pulse_int/prf,'Title','Canceller Output',...
        'YLabel','Magnitude','YLimits',[0 24e-6],...
        'TimeSpanOverrunAction','Scroll');
    for i = 1:numel(filter_coeffs)
        %% MF Scope
        title = strcat("Cancellers order:",string(cancellers_order(i))," output");
        mf_scope.ChannelNames{i} = title;
        %% RTI Definition
        rti{i} = phased.RTIScope('IQDataInput',false,'SampleRate',fs_integrated,...
            'TimeResolution',num_pulse_int/prf,"Name",'Range Time Scope','RangeResolution',range_res,...
            'TimeSpan',1);
    end
    
    mtiBufferIn = [];
    for mtiStep = 1:integratedSteps
        mtiBuffIndex     = buffIndex(mtiStep,integratedIteration);
        mtiBufferIn      = feedLoad(mtiFolder,mtiStep,mtiBuffIndex...
            ,integratedIteration,'mtiBuffer',mtiBufferIn);
        pause(0.01);
        for filterIndex = 1:numel(filter_coeffs)
            sig_disp = squeeze(mtiBufferIn.integratedMtiFiltered(:,mtiBuffIndex,filterIndex));
            rti{filterIndex}(sig_disp);
        end
        % sig_disp =  squeeze(mtiBufferIn.integratedMtiFiltered(:,mtiBuffIndex,:));
        % mf_scope.step(sig_disp);
        drawnow limitrate
        
    end
    
end

function selectRange = batchSelection(batchSize,startIndex)
selectRange = (startIndex-1)*batchSize+1:startIndex*batchSize;
end