classdef rf_experiment < sprf_experiment


    properties

    end

    properties(Dependent)
        integratedSteps;
        integratedBufferSize;
    end

    methods
        function rout = rf_experiment(radar_params,target_params,clutter_params,scene_params)

            rout@sprf_experiment(radar_params,target_params,clutter_params,scene_params)
            rout.logFolderRoot = 'generatedData';
        end


        function obj = mti_process(obj)

            obj = obj.setupOperation(obj.integratedSteps,obj.integratedBufferSize,'clutterProcess',...
                'clutterRxBuff','mti_recursive','mti_recursive_log');
            mtiBuffer.mfMtiFiltered         = zeros(obj.radar_params.samples_per_wave,obj.noIterationBufferSize);
            mtiBuffer.integratedMtiFiltered = zeros(obj.radar_params.samples_per_wave/obj.radar_params.num_pulse_int,...
                obj.integratedBufferSize);
            movAvg = dsp.MovingAverage(obj.radar_params.num_pulse_int,0);
            mtiBufferIn = [];
            currentState = [];
            for mtiStep = 1:obj.integratedSteps
                mtiBufferIn     = obj.bufferLoad(mtiBufferIn);
                selectRange     = batchSelection(obj.radar_params.num_pulse_int,obj.currentBufferIndex);
                cluttered_in    = mtiBufferIn.mf_pulse_cluttered(:,selectRange);
                [filtOut,currentState]  = filter(obj.radar_params.canceller_coeffs,obj.radar_params.canceller_feedback_coeffs,...
                    cluttered_in,currentState,2);
                mtiFilteredData = filtOut;
                mtiBuffer.mfMtiFiltered(:,selectRange)  =  mtiFilteredData ;
                mtiFilteredDs(:,:) = downsample(mtiFilteredData,obj.radar_params.mf_ds_factor);
                integ_mti     =  movAvg(abs(mtiFilteredDs(:,:) )')';
                mtiBuffer.integratedMtiFiltered(:,obj.currentBufferIndex)  =  integ_mti(:) ;
                obj.checkPointSave(mtiBuffer);
                obj = obj.updateState();
            end

        end
        function obj = mti_demo(obj)
            obj = obj.setupOperation(obj.integratedSteps,obj.integratedBufferSize,'mti_recursive',...
                'mtiBuffer');


            mf_scope = timescope('SampleRate',obj.radar_params.fs_integrated,'LayoutDimensions',[1,1],...
                'NumInputPorts',1,'MaximizeAxes','off','Name','MF Output', ...
                'TimeSpanSource','property'...
                );
            set(mf_scope,'ActiveDisplay',1,...
                'TimeSpan',obj.radar_params.num_pulse_int/obj.radar_params.prf,'Title','Canceller Output',...
                'YLabel','Magnitude','YLimits',[0 24e-6],...
                'TimeSpanOverrunAction','Scroll');

            %% MF Scope
            title = ("Recursive doppler canceller(Elliptic Filter with stopband 0.01 x prf):");
            mf_scope.ChannelNames = title;
            %% RTI Definition
            rti = phased.RTIScope('IQDataInput',false,'SampleRate',obj.radar_params.fs_integrated,...
                'TimeResolution',obj.radar_params.num_pulse_int/obj.radar_params.prf,"Name",'Range Time Scope','RangeResolution',obj.radar_params.range_res,...
                'TimeSpan',1);
            mf_pool = parallel.pool.DataQueue;
            mf_pool.afterEach(@(x) mf_scope.step(x));
            rti_pool = parallel.pool.DataQueue;
            rti_pool.afterEach(@(x) rti.step(x));
            mtiBufferIn = [];
            for mtiStep = 1:obj.integratedSteps
                mtiBufferIn = obj.bufferLoad(mtiBufferIn);
                sig_disp = squeeze(mtiBufferIn.integratedMtiFiltered(:,obj.currentBufferIndex));
                rti(sig_disp);
                % mf_scope(sig_disp);
                % send(mf_pool,sig_disp);
                % send(rti_pool,sig_disp);
                drawnow
                obj = obj.updateState();
            end
        end



        function integratedSteps = get.integratedSteps(obj)
            integratedSteps = obj.NSteps/obj.radar_params.num_pulse_int;
        end
        function integratedBufferSize = get.integratedBufferSize(obj)
            integratedBufferSize = obj.noIterationBufferSize/obj.radar_params.num_pulse_int;
        end

    end
end

