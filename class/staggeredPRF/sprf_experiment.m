classdef sprf_experiment
    %SPRF_EXPERIMENT Summary of this class goes here
    %   Detailed explanation goes here

    properties %% Simulation Elements
        waveform phased.RectangularWaveform = phased.RectangularWaveform;
        transmitter phased.Transmitter = phased.Transmitter;
        radiator phased.Radiator = phased.Radiator;
        collector phased.Collector = phased.Collector;
        receiver phased.ReceiverPreamp = phased.ReceiverPreamp;
        sensormotion phased.Platform = phased.Platform;
        tgtmotion phased.Platform = phased.Platform;
        target phased.RadarTarget = phased.RadarTarget;
        channel phased.FreeSpace = phased.FreeSpace;
        clutter constantGammaClutter = constantGammaClutter;
        scene phased.ScenarioViewer = phased.ScenarioViewer;
        radar_params sprf_radar_param = sprf_radar_param;
        target_params sprf_target_param = sprf_target_param;
        clutter_params sprf_clutter_param = sprf_clutter_param;
        scene_params sprf_scene_param = sprf_scene_param;
    end
    properties % simulation flags

    end
    properties
        logFolderRoot = 'generatedData/staggeredPRF'
        simDuration   = sprf_experiment.default_duration;
        TStep   = 1 ;
        noIterationBufferSize = 10000;

    end
    properties (Constant) %% defaults
        default_duration = 20;
    end
    properties (Dependent)
        NSteps;
    end
    properties (Access = protected) % operation variables
        currentTotalSteps;
        currentBufferSize;
        currentBufferIndex;
        currentSourceFolder;
        currentDestinationFolder;
        currentSaveNameTemplate;
        currentStep;
        currentSourceBufferName;
        currentProgress;
        previousReportPercentage;
        % currentSourceBuffer;
        % currentDestinationBuffer;
    end


    methods
        function obj = sprf_experiment(radar_params,target_params,clutter_params,scene_params)
            obj.waveform = phased.RectangularWaveform(...
                'SampleRate',radar_params.fs,'PulseWidth',radar_params.pw,...
                'PRF',radar_params.prf);
            obj.transmitter = phased.Transmitter(...
                'PeakPower',radar_params.peakPower,'InUseOutputPort',true);

            obj.radiator = phased.Radiator(...
                'Sensor',phased.IsotropicAntennaElement,...
                'PropagationSpeed',radar_params.c,'OperatingFrequency',radar_params.fc);

            obj.collector = phased.Collector(...
                'Sensor',phased.IsotropicAntennaElement,...
                'PropagationSpeed',radar_params.c,'OperatingFrequency',radar_params.fc);

            obj.receiver = phased.ReceiverPreamp(...
                'SampleRate',radar_params.fs,'EnableInputPort',true,'NoiseFigure',1.2);

            obj.sensormotion = phased.Platform('InitialPosition',radar_params.position);

            obj.tgtmotion = phased.Platform('InitialPosition',target_params.position,'VelocitySource','Input port');

            obj.target = phased.RadarTarget(...
                'MeanRCS',target_params.rcs,'PropagationSpeed',radar_params.c,'OperatingFrequency',radar_params.fc);

            obj.channel = phased.FreeSpace(...
                'PropagationSpeed',radar_params.c,'OperatingFrequency',radar_params.fc,...
                'TwoWayPropagation',true,'SampleRate',radar_params.fs);
            obj.clutter   = constantGammaClutter('Sensor',obj.collector.Sensor,...
                'PropagationSpeed',obj.radiator.PropagationSpeed,...
                'OperatingFrequency',obj.radiator.OperatingFrequency,...
                'SampleRate',obj.waveform.SampleRate,'TransmitSignalInputPort',false,...h
                'PRF',obj.waveform.PRF,'Gamma',clutter_params.gamma,'PlatformHeight',radar_params.position(3),...
                'TransmitERP',radar_params.peakPower,'PlatformSpeed',0,'PlatformDirection',[0;0],...
                'MountingAngles',[0 0 0],'ClutterMinRange',0,'ClutterMaxRange',2000,...
                'ClutterAzimuthSpan',clutter_params.azimuthSpan,'NumPulses',clutter_params.numPulse,'PatchAzimuthSpan',clutter_params.patchAzimuthSpan,...
                'SeedSource','Property','Seed',2011);

            obj.scene = phased.ScenarioViewer( ...
                'BeamWidth',scene_params.beamWidth,...
                'BeamRange',scene_params.beamRange,'UpdateRate',scene_params.updateRate,...
                'PlatformNames',scene_params.platformNames,'ShowPosition',true,...
                'ShowSpeed',true,'ShowAltitude',true,'ShowLegend',true,...
                'TrailLength',scene_params.trailLength,'ReducePlotRate',true);
            obj = obj.setSimParams(radar_params,target_params,clutter_params,scene_params);
        end
        function obj = setSimParams(obj,radar_params,target_params,clutter_params,scene_params)
            obj.TStep = 10/radar_params.prf;
            obj.radar_params = radar_params;
            obj.target_params = target_params;
            obj.clutter_params = clutter_params;
            obj.scene_params = scene_params;

        end

        

        function obj = mosim(obj)
            % initialize operation
            initFolder(obj.currentDestinationFolder,true);
            obj = obj.setupOperation(obj.NSteps,obj.noIterationBufferSize,[],...
                [],'mosim','mosim_log');
            % initialize buffers
            motionSimBuffer.tgt_pos = zeros(3,obj.noIterationBufferSize);
            motionSimBuffer.tgt_ang = zeros(1,obj.noIterationBufferSize);
            motionSimBuffer.tgt_vel = zeros(3,obj.noIterationBufferSize);
            motionSimBuffer.rad_ang = zeros(2,obj.noIterationBufferSize);
            motionSimBuffer.rad_vel = zeros(3,obj.noIterationBufferSize);
            motionSimBuffer.rad_pos = zeros(3,obj.noIterationBufferSize);
            % simulation
            init_tgt_vel = obj.target_params.velocity;
            for curStep = 1:obj.currentTotalSteps
                obj = obj.updateState();
                if obj.currentStep ==1 % Need to initialize target velocity to be used in current iteration
                    [motionSimBuffer.tgt_pos(:,obj.currentBufferIndex)] = obj.tgtmotion(obj.TStep,init_tgt_vel);
                elseif obj.currentBufferIndex == 1
                    [motionSimBuffer.tgt_pos(:,obj.currentBufferIndex)] = obj.tgtmotion(obj.TStep,motionSimBuffer.tgt_vel(:,obj.currentBufferSize));
                else
                    [motionSimBuffer.tgt_pos(:,obj.currentBufferIndex)] = obj.tgtmotion(obj.TStep,motionSimBuffer.tgt_vel(:,obj.currentBufferIndex-1));
                end
                motionSimBuffer.tgt_ang(:,obj.currentBufferIndex) = spinningAngle(obj.target_params.spin_center,motionSimBuffer.tgt_pos(:,obj.currentBufferIndex));
                motionSimBuffer.tgt_vel(:,obj.currentBufferIndex) = spinningVelocity(obj.target_params.angular_speed,obj.target_params.spin_radius,motionSimBuffer.tgt_ang(:,obj.currentBufferIndex));
                [motionSimBuffer.rad_pos(:,obj.currentBufferIndex),motionSimBuffer.rad_vel(:,obj.currentBufferIndex)] =obj.sensormotion(obj.TStep);
                [~,motionSimBuffer.rad_ang(:,obj.currentBufferIndex)] = rangeangle(motionSimBuffer.tgt_pos(:,obj.currentBufferIndex));
                obj.checkPointSave(motionSimBuffer);
            end


        end
        function obj = setupOperation(obj,varargin)
            %% setup save folder load folder etc
            %% 1.totalsteps 2.bufferSize 3.sourceFolder([] if no load) 4.sourceBufferName 5.destinationfoldername 5.saveFileName
            totalSteps = varargin{1};
            bSize = varargin{2};
            if nargin>3 %% load
                sourcefolder = varargin{3};
                sourceBufferName = varargin{4};
                if ~isempty(sourcefolder)
                    obj.currentSourceFolder = sourcefolder;
                    obj.currentSourceBufferName = sourceBufferName;
                end
            end
            if nargin > 5 %% save
                destinationFolderName = varargin{5};
                saveFileName = varargin{6};
                [obj.currentDestinationFolder,obj.currentSaveNameTemplate] ...
                    = saveFolderDef(obj.logFolderRoot,destinationFolderName,saveFileName);
                initFolder(obj.currentDestinationFolder,true);

            end
            %% accounting indices
            obj.currentProgress = 0;
            obj.previousReportPercentage = 0;
            obj.currentTotalSteps = totalSteps;
            obj.currentBufferSize = bufferSize(totalSteps,bSize);
            obj.currentBufferIndex = 1;
            obj.currentStep = 1;
            % obj.currentSourceBuffer = [];
            % obj.currentDestinationBuffer = [];


        end
        function obj = updateState(obj)
            %% update indices
            curStep = obj.currentStep + 1;
            obj.currentBufferIndex = buffIndex(curStep,obj.currentBufferSize);
            obj.currentStep = curStep;
            obj.currentProgress = obj.currentStep/obj.currentTotalSteps*100;
            if obj.currentProgress - obj.previousReportPercentage > 1
                obj.previousReportPercentage = obj.currentProgress;
                fprintf('Progress at %.1f percent...\n',obj.currentProgress);
            end
            
        end
        function checkPointSave(obj,buffer)
            buffName = inputname(2);
            feedSave(obj.currentSaveNameTemplate,obj.currentStep,obj.currentTotalSteps,buffer,obj.currentBufferIndex,obj.currentBufferSize,buffName);
        end
        function buffer = bufferLoad(obj,bufferIn)
            buffer = feedLoad(obj.currentSourceFolder,obj.currentStep,obj.currentBufferIndex,obj.currentBufferSize,obj.currentSourceBufferName,bufferIn);
        end
        %% get Functions
        function Nsteps = get.NSteps(obj)
            Nsteps = obj.simDuration/obj.TStep;
        end
    end
end
function setBuffer(varName,buffer)
    assignin('caller',varName,buffer);
end
