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
        tempOffset;                             % Coefficient for temperature depending offset
        error2dOffset;                          % first column: inputArg, second column: relativeArg, third column: error
    end
    
    methods (Access = 'public')
        % Main methods for all sensors, here new sensors are created and
        % initialized; the function "sens" is defined here

        function obj = SensorNew(is3D_input, isFaulty)
            % creating a new sensor

            % adding Sensor3D properties if is3D == true
            obj.addprop("is3D");
            if is3D_input
                obj.is3D = true;

                obj.addprop("offsetX");
                obj.addprop("offsetY");
                obj.addprop("offsetZ");
                obj.addprop("walkDiffusionCoef");
                obj.addprop("transMatrix");

                obj.addprop("stateWalkX");
                obj.addprop("stateWalkY");
                obj.addprop("stateWalkZ");
                obj.stateWalkX = 0;
                obj.stateWalkY = 0;
                obj.stateWalkZ = 0;
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
        function [outputArg1, outputArg2, outputArg3] = sens(obj,temp,inputArg1,inputArg2,inputArg3)
            if nargin == 3
                inputArg1 = obj.addOffset(inputArg1);
                inputArg1 = obj.add2DOffset(inputArg1,temp);
                inputArg1 = obj.addTempOffset(inputArg1,temp);
                inputArg1 = obj.whiteNoise(inputArg1);
                inputArg1 = obj.quantization(inputArg1);
                inputArg1 = obj.saturation(inputArg1);
                outputArg1 = inputArg1;
                outputArg2 = NaN;
                outputArg3 = NaN;
            elseif nargin == 5
                [inputArg1,inputArg2,inputArg3] = obj.tranformAxis(inputArg1,inputArg2,inputArg3);
                [inputArg1,inputArg2,inputArg3] = obj.addOffset3D(inputArg1,inputArg2,inputArg3);
                [inputArg1,inputArg2,inputArg3] = obj.randomWalk(inputArg1,inputArg2,inputArg3);

                inputArg1 = obj.addOffset(inputArg1);
                inputArg2 = obj.addOffset(inputArg2);
                inputArg3 = obj.addOffset(inputArg3);

                inputArg1 = obj.add2DOffset(inputArg1,temp);
                inputArg2 = obj.add2DOffset(inputArg2,temp);
                inputArg3 = obj.add2DOffset(inputArg3,temp);

                inputArg1 = obj.addTempOffset(inputArg1,temp);
                inputArg2 = obj.addTempOffset(inputArg2,temp);
                inputArg3 = obj.addTempOffset(inputArg3,temp);

                inputArg1 = obj.whiteNoise(inputArg1);
                inputArg2 = obj.whiteNoise(inputArg2);
                inputArg3 = obj.whiteNoise(inputArg3);

                inputArg1 = obj.quantization(inputArg1);
                inputArg2 = obj.quantization(inputArg2);
                inputArg3 = obj.quantization(inputArg3);

                inputArg1 = obj.saturation(inputArg1);
                inputArg2 = obj.saturation(inputArg2);
                inputArg3 = obj.saturation(inputArg3);

                outputArg1 = inputArg1;
                outputArg2 = inputArg2;
                outputArg3 = inputArg3;
            else
                error("Invalid number of inputs")
            end
        end
    end

    methods (Access = 'protected')
        % Main protected methods, here all the internal functions are
        % located

        function [outputArg1,outputArg2,outputArg3] = addOffset3D(obj,inputArg1,inputArg2,inputArg3)            
            if (~isempty(obj.offsetX))
                inputArg1 = inputArg1 + ones(size(inputArg1)).*obj.offsetX;
            end
            if (~isempty(obj.offsetY))
                inputArg2 = inputArg2 + ones(size(inputArg2)).*obj.offsetY;
            end
            if (~isempty(obj.offsetZ))
                inputArg3 = inputArg3 + ones(size(inputArg3)).*obj.offsetZ;
            end
            outputArg1 = inputArg1;
            outputArg2 = inputArg2;
            outputArg3 = inputArg3;
        end

        function [outputArg1,outputArg2,outputArg3] = randomWalk(obj,inputArg1,inputArg2,inputArg3)
            if (~isempty(obj.walkDiffusionCoef))
                % distance
                s = sqrt ( 2.0 * 3 * obj.walkDiffusionCoef * obj.dt ) * randn ( 1, 1);

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
                obj.stateWalkX=obj.stateWalkX+dx;
                obj.stateWalkY=obj.stateWalkY+dy;
                obj.stateWalkZ=obj.stateWalkZ+dz;
                
            end
            
            outputArg1=inputArg1+obj.stateWalkX;
            outputArg2=inputArg2+obj.stateWalkY;
            outputArg3=inputArg3+obj.stateWalkZ;
        end

        function [outputArg1,outputArg2,outputArg3] = tranformAxis(obj,inputArg1,inputArg2,inputArg3)
            if (~isempty(obj.transMatrix))
                transOut=obj.transMatrix*[inputArg1,inputArg2,inputArg3]';
                inputArg1 = transOut(1);
                inputArg2 = transOut(2);
                inputArg3 = transOut(3);
            end
            
            outputArg1 = inputArg1;
            outputArg2 = inputArg2;
            outputArg3 = inputArg3;
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

