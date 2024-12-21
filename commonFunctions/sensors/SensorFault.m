classdef SensorFault < handle
    
    % Author: Jan Hammelman
    % Skyward Experimental Rocketry | ELC-SCS Dept | electronics@skywarder.eu
    % email: jan.hammelmann@skywarder.eu,alessandro.delduca@skywarder.eu
    % Release date: 01/03/2021
    
    %SENSOR Super call for all sensor
    %   Inherit from handle to have state variables which can be changed
    %   inside a method
    % update: Alessandro Donadi, 31/08/2023
    %NOTE ON THE UPDATE:
    %   Added fault simulation capabilities from the faliure simulator made
    %   by Angelo G. Gaillet and released on 31/07/2022
    properties (Access='public')
        minMeasurementRange; % Max limit of sensor
        maxMeasurementRange; % Min limit of sensor
        
        bit; % number of bits for the sensor ( if available)
        resolution; % resolution of the sensor
        
        % noises
        noiseType;                              % White (default) or Pink
        noiseDataTrack1;
        noiseFactor;
        noiseVariance; % Varianze for the gaussian white noise
        
        offset; % Offset in all directions
        
        tempOffset; % Coefficent for temperature depending offset
        
        dt; % Sampling time
        
        error2dOffset; % first column: inputArg, second column: relativeArg, third column: error

        fault_time; % if set to -1 it will be randomly chosen between a min and a max time in seconds

        max_fault_time; %upper bound of the time window where the fault can occur in case it's randomly chosen

        min_fault_time; %lower bound of the time window where the fault can occur in case it's randomly chosen
    end

    properties (Access = public) %attributes used for fault simulation
        failureType;
        fault_offset;
        lambda;
        sigmaDeg;
        sigmaIS;
        tError;
        gain;
        value;
        likelihoodFS;
        likelihoodIS;
        alsoNegSpikes;
        severity;
        randomSpikeAmpl;
        satMax;
        satMin;
        settings;

        frozenValue;
    end
    
    
    methods (Access='public')
        function obj = SensorFault()
            %SENSOR Construct an instance of this class
            %   Sensor is the Superclass of all sensors
            obj = obj.reset();
        end
        
        function outputArg = sens(obj,inputArg,temp,t)
            %SENS Method to use the sensor.
            %   Gets the simulation data and extract the unideal sensor
            %   output
            %
            %  Inputs:
            %  inputArg sensor data
            %  temp: temperature of the sensor
            %  
            %  Outputs:
            %  outputArg sensor data with noise,
            %  offsets, etc.
            % N.B. this offsets and saturations ecc... are inherent
            % non-idealities of the simulated sensors, not FAULTS, with
            % faults in this context we mean SUDDEN non-idealities, and not
            % a characteristic of the transducer or sensor in question that
            % we already know it's present. 
            inputArg=obj.add2DOffset(inputArg,temp);
            inputArg=obj.addNoise(inputArg,t);
            inputArg=obj.addOffset(inputArg);
            inputArg=obj.addTempOffset(inputArg,temp);
            inputArg=obj.quantization(inputArg);
            inputArg=obj.saturation(inputArg); 
            outputArg = inputArg;
        end

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
                        %sensorData(i) = obj.frozenValue;
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
    
        function obj = setIncrementalSpiking(obj, sigma, likelihood, severity, alsoNegative, random)
            if islogical(random)
                if islogical(alsoNegative)
                    if (likelihood > 0 && likelihood  < 1)
                        obj.sigmaIS = sigma;
                        obj.likelihoodIS = likelihood;
                        obj.severity = severity;
                        obj.alsoNegSpikes = alsoNegative;
                        obj.settings.is = true;
                        obj.randomSpikeAmpl = random;
                    else
                        error('The likelihood must be between 0 and 1')
                    end
                else
                    error('alsoNegative must be either False (for only positive spikes) or True (for both positive and negative spikes)');
                end
            else
                error('random must be either False (for randomness in the increments) or True (for fixed increments)');
            end
        end
    
        function obj = setSensorSaturation(obj, min, max)
            obj.satMax = max;
            obj.satMin = min;
            obj.settings.saturateFlag = True;
        end
      
    end
    
    
    methods (Access='protected')
        function outputArg = saturation(obj,inputArg)
            %SATURATION Includes saturation to the sensor model
            %   inputArg is limited to minMeasurementRange at the lower end and maxMeasurementRange at the upper end
            %
            %  Necessary properties:
            %  minMeasurementRange: Max limit of sensor
            %  maxMeasurementRange: Min limit of sensor
            %
            %  Inputs:
            %  inputArg: sensor data
            %  
            %  Outputs:
            %  outputArg: sensor data with saturation
            
            % checks if sensor data is higher than max possible value
            if (~isempty(obj.maxMeasurementRange))
                inputArg(inputArg>obj.maxMeasurementRange)=obj.maxMeasurementRange;
            end
            
            % checks if sensor data is lower than min possible value
            if (~isempty(obj.minMeasurementRange))
                inputArg(inputArg<obj.minMeasurementRange)=obj.minMeasurementRange;
            end
            
            outputArg = inputArg;
        end
        
        
        function outputArg = quantization(obj,inputArg)
            %QUATIZATION Quantize the sensor data
            %   Quantizes the signal with the resolution properie of the object
            %
            %  Necessary properties:
            %  resolution: resolution of the sensor
            %
            %  Inputs:
            %  inputArg: sensor data
            %  
            %  Outputs:
            %  outputArg: quantized sensor data
            
            if (~isempty(obj.resolution))
                inputArg = obj.resolution*round(inputArg/obj.resolution);
            end
            outputArg = inputArg;
        end
        
        
        function outputArg = addNoise(obj,inputArg,t)
            %WHITE_NOISE Includes gaussian white noise to the sensor data
            %   Adds gaussian white noise with variance noiseVariance
            %
            %  Necessary properties:
            %  noiseVariance: Varianze for the gaussian white noise
            %
            %  Inputs:
            %  inputArg: sensor data
            %  
            %  Outputs:
            %  outputArg: sensor data with white noise
            
            % if (~isempty(obj.noiseVariance))
            %     %inputArg=inputArg+ones(size(inputArg)).*sqrt(obj.noiseVariance).*randn(1,1);,
            %     inputArg=inputArg+sqrt(obj.noiseVariance).*randn(size(inputArg));
            % end
            % outputArg = inputArg;

            if ~isempty(obj.noiseVariance)              % check for old results
                inputArg = inputArg + sqrt(obj.noiseVariance).*randn(length(inputArg),1);
            elseif ~isempty(obj.noiseDataTrack1)    
                if strcmp(obj.noiseType, "white")
                    inputArg = inputArg + sqrt(obj.noiseDataTrack1*obj.noiseFactor).*randn(length(inputArg),1);
                elseif strcmp(obj.noiseType, "pink")
                    for ii = 1:length(obj.noiseDataTrack1.peaks_vect_f)
                        inputArg = inputArg + obj.noiseDataTrack1.peaks_vect_val(ii)*obj.noiseFactor*sin(2*pi*obj.noiseDataTrack1.peaks_vect_f(ii)*t + randn(1));
                    end
                    inputArg = inputArg + sqrt(obj.noiseDataTrack1.variance*obj.noiseFactor).*randn(length(inputArg),1);
                else
                    error("This noise is not defined")
                end
            end

            outputArg = inputArg;
        end
                
        
        function outputArg = addOffset(obj,inputArg)
            %ADD_OFFSET Adds an offset to the signal
            %   adds the propertie offset to the input signal
            %
            %  Necessary properties:
            %  offset: Offset in all directions
            %
            %  Inputs:
            %  inputArg: sensor data
            %  
            %  Outputs:
            %  outputArg: sensor data plus offset
            
            if (~isempty(obj.offset))
                inputArg=inputArg+ones(size(inputArg)).*obj.offset;
            end
            outputArg = inputArg;
        end
        
        
        function outputArg = addTempOffset(obj,inputArg,temp)
            %ADD_Temp_OFFSET Adds an temperature dependen offset to the signal
            %   adds the propertie tempOffset multiplied by the temperature input temp to the input signal
            %
            %  Necessary properties:
            %  tempOffset: Coefficent for temperature depending offset
            %
            %  Inputs:
            %  inputArg: sensor data
            %  temp: temperature of the sensor
            %  
            %  Outputs:
            %  outputArg: sensor data plus temerature depending offset
            
            if (~isempty(obj.tempOffset))
                inputArg=inputArg+ones(size(inputArg)).*temp*obj.tempOffset;
            end
            outputArg = inputArg;
        end
        
        
        function outputArg = add2DOffset(obj,inputArg,temp)
            %ADD_2D_OFFSET Adds a offset that depends on the inputArg and temp
            %  The offset is linear interpolated with the data in matrice error2dOffset
            %
            %  Necessary properties:
            %  error2dOffset: data for offset with first column: inputArg, second column: relativeArg, third column: error
            %
            %  Inputs:
            %  inputArg: sensor data
            %  temp: temperature of the sensor
            %  
            %  Outputs:
            %  outputArg: sensor data plus 2D depending offset
            
            if (~isempty(obj.error2dOffset))
                inputArg=inputArg+...
                    griddata(obj.error2dOffset(:,1),obj.error2dOffset(:,2),obj.error2dOffset(:,3),inputArg,temp);
            end
            outputArg = inputArg;
        end
        
        function obj = reset(obj) %method
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

    
    end
    
    methods (Access = private)
    
        function data = saturate(data)
            data = max(obj.satMin, min(obj.satMax, data));
        end
    
    end
end

