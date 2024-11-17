classdef SensorNew < dynamicprops

    % Author: Stefano Belletti
    % Skyward Experimental Rocketry | AVN - GNC
    % email: stefano.belletti@skywarder.eu
    % Release date: 17/11/2024
    % 
    % SENSOR Super call for all sensor
    % 
    % Creating a new sensor: [sensor obj] = Sensor(is3D, isFaulty)
    % 
    % INPUT:        DIMENSIONS:     TYPE:
    % is3D          [1x1]           logical
    % isFaulty      [1x1]           logical

    % properties
    properties (Access = 'public')
        minMeasurementRange;                    % Max limit of sensor
        maxMeasurementRange;                    % Min limit of sensor
        bit;                                    % Number of bits for the sensor (if available)
        resolution;                             % resolution of the sensor (set internally if bit is available)
        dt;                                     % Sampling time

        % noises
        noiseVariance;                          % Defining gaussian white noise
        
        % offset
        offset;                                 % Offset in all directions
        tempOffset;                             % Coefficent for temperature depending offset
        error2dOffset;                          % first column: inputArg, second column: relativeArg, third column: error
    end
    
    methods (Access = 'public')
        % Main methods for all sensors, here new sensors are created and
        % initialized; the function "sens" is defined here

        function obj = SensorNew(is3D, isFaulty)
            % creating a new sensor

            % adding Sensor3D properties if is3D == true
            obj.addprop("is3D");
            if is3D
                obj.is3D = true;

                obj.addprop("TD_offset");
                obj.addprop("TD_walkDiffusionCoef");
                obj.addprop("TD_transMatrix");

                obj.addprop("TD_stateWalk");
                obj.TD_stateWalk.x = 0;
                obj.TD_stateWalk.y = 0;
                obj.TD_stateWalk.z = 0;
            else
                obj.is3D = false;
            end

            % fault
            obj.addprop("isFaulty");
            if isFaulty
                obj.isFaulty = true;

                obj.addprop("fault_time");
                obj.addprop("max_fault_time");
                obj.addprop("min_fault_time");
                obj.addprop("failureType");
                obj.addprop("fault_offset");
                obj.addprop("lambda");
                obj.addprop("sigmaDeg");
                obj.addprop("sigmaIS");
                obj.addprop("tError");
                obj.addprop("gain");
                obj.addprop("value");
                obj.addprop("likelihoodFS");
                obj.addprop("likelihoodIS");
                obj.addprop("alsoNegSpikes");
                obj.addprop("severity");
                obj.addprop("randomSpikeAmpl");
                obj.addprop("satMax");
                obj.addprop("satMin");
                obj.addprop("settings");
                obj.addprop("frozenValue");

                obj = obj.fault_reset();
            else
                obj.isFaulty = false;
            end
        end
        
        % sens function
        function outputArg = sens(obj,inputArg,temp)
            if ~isstruct(inputArg)
                inputArg = obj.addOffset(inputArg);
                inputArg = obj.add2DOffset(inputArg,temp);
                inputArg = obj.addTempOffset(inputArg,temp);
                inputArg = obj.whiteNoise(inputArg);
                inputArg = obj.quantization(inputArg);
                inputArg = obj.saturation(inputArg);
                outputArg = inputArg;
            else
                inputArg = obj.tranformAxis(inputArg);          
                inputArg = obj.addOffset3D(inputArg);           
                inputArg = obj.randomWalk(inputArg);            

                inputArg.x = obj.addOffset(inputArg.x);
                inputArg.y = obj.addOffset(inputArg.y);
                inputArg.z = obj.addOffset(inputArg.z);

                inputArg.x = obj.add2DOffset(inputArg.x,temp);
                inputArg.y = obj.add2DOffset(inputArg.y,temp);
                inputArg.z = obj.add2DOffset(inputArg.z,temp);

                inputArg.x = obj.addTempOffset(inputArg.x,temp);
                inputArg.y = obj.addTempOffset(inputArg.y,temp);
                inputArg.z = obj.addTempOffset(inputArg.z,temp);

                inputArg.x = obj.whiteNoise(inputArg.x);
                inputArg.y = obj.whiteNoise(inputArg.y);
                inputArg.z = obj.whiteNoise(inputArg.z);

                inputArg.x = obj.quantization(inputArg.x);
                inputArg.y = obj.quantization(inputArg.y);
                inputArg.z = obj.quantization(inputArg.z);

                inputArg.x = obj.saturation(inputArg.x);
                inputArg.y = obj.saturation(inputArg.y);
                inputArg.z = obj.saturation(inputArg.z);

                outputArg.x = inputArg.x;
                outputArg.y = inputArg.y;
                outputArg.z = inputArg.z;
            end
        end
    end

    methods (Access = 'protected')
        % Main protected methods, here all the internal functions are
        % located

        function [outputArg] = addOffset3D(obj,inputArg)            
            if (~isempty(obj.TD_offset.x))
                inputArg.x = inputArg.x + ones(size(inputArg.x)).*obj.TD_offset.x;
            end
            if (~isempty(obj.TD_offset.y))
                inputArg.y = inputArg.y + ones(size(inputArg.y)).*obj.TD_offset.y;
            end
            if (~isempty(obj.TD_offset.z))
                inputArg.z = inputArg.z + ones(size(inputArg.z)).*obj.TD_offset.z;
            end
            outputArg.x = inputArg.x;
            outputArg.y = inputArg.y;
            outputArg.z = inputArg.z;
        end

        function [outputArg] = randomWalk(obj,inputArg)
            if (~isempty(obj.TD_walkDiffusionCoef))
                % distance
                s = sqrt ( 2.0 * 3 * obj.TD_walkDiffusionCoef * obj.dt ) * randn ( 1, 1);

                % direction
                x=randn(1,1);
                y=randn(1,1);
                z=randn(1,1);

                % normalize
                v=s/sqrt(x^2+y^2+z^2);
                dx=x*v;
                dy=y*v;
                dz=z*v;

                % sum random walk
                obj.TD_stateWalk.x = obj.TD_stateWalk.x + dx;
                obj.TD_stateWalk.y = obj.TD_stateWalk.y + dy;
                obj.TD_stateWalk.z = obj.TD_stateWalk.z + dz;
                
            end
            
            outputArg.x = inputArg.x + obj.TD_stateWalk.x;
            outputArg.y = inputArg.y + obj.TD_stateWalk.y;
            outputArg.z = inputArg.z + obj.TD_stateWalk.z;
        end

        function [outputArg] = tranformAxis(obj,inputArg)
            if (~isempty(obj.TD_transMatrix))
                transOut=obj.TD_transMatrix*[inputArg.x;inputArg.y;inputArg.z];
                inputArg.x = transOut(1);
                inputArg.y = transOut(2);
                inputArg.z = transOut(3);
            end
            
            outputArg.x = inputArg.x;
            outputArg.y = inputArg.y;
            outputArg.z = inputArg.z;
        end

        function outputArg = addOffset(obj,inputArg)
            if (~isempty(obj.offset))
                inputArg=inputArg+ones(size(inputArg)).*obj.offset;
            end
            outputArg = inputArg;
        end

        function outputArg = add2DOffset(obj,inputArg,temp)            
            if (~isempty(obj.error2dOffset))
                inputArg=inputArg + griddata(obj.error2dOffset(:,1),obj.error2dOffset(:,2),obj.error2dOffset(:,3),inputArg,temp);
            end
            outputArg = inputArg;
        end

        function outputArg = addTempOffset(obj,inputArg,temp)
            if (~isempty(obj.tempOffset))
                inputArg=inputArg+ones(size(inputArg)).*temp*obj.tempOffset;
            end
            outputArg = inputArg;
        end

        function outputArg = whiteNoise(obj,inputArg)
            if (~isempty(obj.noiseVariance))
                inputArg=inputArg+sqrt(obj.noiseVariance).*randn(length(inputArg),1);
            end
            outputArg = inputArg;
        end

        function outputArg = quantization(obj,inputArg)
            if isempty(obj.resolution)
                if (~isempty(obj.maxMeasurementRange)) && (~isempty(obj.minMeasurementRange)) && (~isempty(obj.bit))
                    obj.resolution = (obj.maxMeasurementRange - obj.minMeasurementRange)/(2^obj.bit);
                end
            end
            inputArg = obj.resolution*round(inputArg/obj.resolution);
            outputArg = inputArg;
        end

        function outputArg = saturation(obj,inputArg)                        
            % checks if sensor data is lower than min possible value
            if (~isempty(obj.minMeasurementRange))
                inputArg(inputArg<obj.minMeasurementRange)=obj.minMeasurementRange;
            end
            % checks if sensor data is higher than max possible value
            if (~isempty(obj.maxMeasurementRange))
                inputArg(inputArg>obj.maxMeasurementRange)=obj.maxMeasurementRange;
            end
            
            outputArg = inputArg;
        end

        % faulty sensors
        function obj = fault_reset(obj)
            obj.failureType = 'None';
            obj.fault_offset = 0;
            obj.lambda = 0;
            obj.sigmaDeg = 0;
            obj.sigmaIS = 0;
            obj.tError = 0;
            obj.gain = 1;
            obj.value = 0;
            obj.likelihoodFS = 0;
            obj.likelihoodIS = 0;
            obj.alsoNegSpikes = false;
            obj.severity = 0;
            obj.randomSpikeAmpl = false;
            obj.satMax = inf;
            obj.satMin = -inf;
            obj.settings.saturateFlag = false;
            obj.settings.bias = false;
            obj.settings.drift = false;
            obj.settings.degradation = false;
            obj.settings.freezing = false;
            obj.settings.calerr = false;
            obj.settings.fs = false;
            obj.settings.is = false;
            
            obj.frozenValue = nan;
        end

        function data = saturate(data)
            data = max(obj.satMin, min(obj.satMax, data));
        end
    end

    methods (Access = 'public')
        % Main methods for faulty sensors

        function obj = getFaultTime(obj) % to set when the fault occurs
            if obj.fault_time == -1
                obj.fault_time = randi((obj.max_fault_time-obj.min_fault_time)*10)/10 + obj.min_fault_time;
            end
        end

        function [obj, tError] = setErrorTime(obj) % to set when the fault occurs
            obj.getFaultTime()
            if obj.fault_time >= 0
                obj.tError = obj.fault_time;
                tError = obj.fault_time;
            else
                error('Time needs to be positive')
            end
        end

        function [obj, sensorData] = applyFailure(obj, sensorData, timestamp) %function necessary to set a fault at a certain timestamp of the simulation, the operation applied depends on the simulated fault
            for i = 1:length(timestamp)
                if timestamp(i) >= obj.tError
                    if obj.settings.bias
                        sensorData(i) = sensorData(i) + obj.fault_offset;
                    end
                    if obj.settings.drift
                        sensorData(i) = sensorData(i) + obj.lambda * (timestamp(i) - obj.tError);
                    end
                    if obj.settings.degradation
                        sensorData(i) = sensorData(i) - obj.sigmaDeg + 2*obj.sigmaDeg*rand;
                    end
                    if obj.settings.freezing
                        % if timestamp(i) == obj.tError
                        %     obj.frozenValue = sensorData(i);
                        % end
                        sensorData(i) = obj.frozenValue;
                    end
                    if obj.settings.calerr
                        sensorData(i) = sensorData(i)*obj.gain;
                    end
                    if obj.settings.fs
                        if rand > (1 - obj.likelihoodFS)
                            sensorData(i) = obj.value;
                        end
                    end
                    if obj.settings.is
                        if rand > (1 - obj.likelihoodIS)
                            if obj.randomSpikeAmpl == true
                                randVal = rand;
                            else
                                randVal = 1;
                            end
                            if obj.alsoNegSpikes
                                sensorData(i) = sensorData(i) -obj.sigmaIS + 2*obj.sigmaIS*randVal;
                            else
                                sensorData(i) = sensorData(i) +obj.sigmaIS*randVal;
                            end
                        end
                    end
                else
                    if obj.settings.freezing
                        obj.frozenValue = sensorData(i);
                        % sensorData(i) = obj.frozenValue;
                    end
                end
            end
    
            if obj.settings.saturateFlag
                sensorData = obj.saturate(sensorData);
            end
    
        end

        function obj = setOffset(obj, fault_offset)
            obj.fault_offset = fault_offset;
            obj.settings.bias = true;
        end
    
        function obj = setDrift(obj, lambda)
            obj.lambda = lambda;
            obj.settings.drift = true;
        end
    
        function obj = setDegradation(obj, sigma)
            obj.sigmaDeg = sigma;
            obj.settings.degradation = true;
        end
    
        function obj = setFreezing (obj)
            obj.settings.freezing = true;
        end
    
        function obj = setCalibrationError(obj, gain)
            obj.gain = gain;
            obj.settings.calerr = true;
        end
    
        function obj = setFixedSpiking(obj, value, likelihood)
            if (likelihood > 0 && likelihood  < 1)
                obj.value = value;
                obj.likelihoodFS = likelihood;
                obj.settings.fs = true;
            else
                error('The likelihood must be between 0 and 1')
            end
        end
    end
end

