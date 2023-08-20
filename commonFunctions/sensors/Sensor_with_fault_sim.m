classdef Sensor_with_fault_sim < handle
    
    % Author: Jan Hammelman
    % Skyward Experimental Rocketry | ELC-SCS Dept | electronics@skywarder.eu
    % email: jan.hammelmann@skywarder.eu,alessandro.delduca@skywarder.eu
    % Release date: 01/03/2021
    
    %SENSOR Super call for all sensor
    %   Inherit from handle to have state variables which can be changed
    %   inside a method
    
    properties (Access='public')
        minMeasurementRange; % Max limit of sensor
        maxMeasurementRange; % Min limit of sensor
        
        bit; % number of bits for the sensor ( if available)
        resolution; % resolution of the sensor
        
        noiseVariance; % Varianze for the gaussian white noise
        
        offset; % Offset in all directions
        
        tempOffset; % Coefficent for temperature depending offset
        
        dt; % Sampling time
        
        error2dOffset; % first column: inputArg, second column: relativeArg, third column: error

        twoPhaseNoise; %simulates different noises for ascent and descent

        secondPhase; %flag for changing the noise profile after reaching the apogee

    end

    properties (Access = private) %attributes private
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
        function obj = Sensor_with_fault_sim()
            %SENSOR Construct an instance of this class
            %   Sensor is the Superclass of all sensors
            obj = obj.reset();
        end
        
        function outputArg = sens(obj,inputArg,temp)
            %SENS Method to use the sensor.
            %   Gets the simulation data and extract the unideal sensor
            %   output
            %
            %  Inputs:
            %  inputArg sensor data
            %  temp: temperature of the sensor
            %  
            %  Outputs:
            %  outputArg sensor data with nois,
            %  offsets, etc.
            
            inputArg=obj.add2DOffset(inputArg,temp);
            inputArg=obj.whiteNoise(inputArg);
            inputArg=obj.addOffset(inputArg);
            inputArg=obj.addTempOffset(inputArg,temp);
            inputArg=obj.quantization(inputArg);
            inputArg=obj.saturation(inputArg); 
            outputArg = inputArg;
        end
                function obj = setErrorTime(obj, startTimestamp)
            if startTimestamp >= 0
                obj.tError = startTimestamp;
            else
                error('Time needs to be positive')
            end
        end
    
        function [obj, sensorData] = applyFailure(obj, sensorData, timestamp)
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
                        if timestamp(i) == obj.tError
                            obj.frozenValue = sensorData(i);
                        end
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
        
        
        function outputArg = whiteNoise(obj,inputArg)
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
            noiseAmp = 1.5; % Amplitude of the white noise
            lowerAmp = 0.1;
            cutOffFreq1 = 1; % Cut-off frequency of the low-pass filter for ascension
            cutOffFreq2 = 3; % Cut-off frequency of the low-pass filter for desention
            normalizedCutOffFreq1 = cutOffFreq1 / (0.5 * 50); % Normalize the cut-off frequency
            normalizedCutOffFreq2 = cutOffFreq2 / (0.5 * 50); % Normalize the cut-off frequency
            if (~isempty(obj.noiseVariance) && isequal(obj.twoPhaseNoise, false))
            %   inputArg=inputArg+ones(size(inputArg)).*sqrt(obj.noiseVariance).*randn(1,1);,
                inputArg=inputArg+sqrt(obj.noiseVariance).*randn(size(inputArg));
            else
                noise_sensor = noiseAmp * randn(size(inputArg));
                Backgroundnoise_sensor = lowerAmp * randn(size(inputArg)); % Generate white noise with the same size as the sensor signal
                if isequal(obj.secondPhase, false)
                    [b2, a2] = butter(4, normalizedCutOffFreq2, 'low'); % Butterworth low-pass filter coefficients
                     filteredNoise = filter(b2, a2, noise_sensor); % Apply the low-pass filter to the white noise
                else % caso in cui è nella prima fase
                    [b1, a1] = butter(4, normalizedCutOffFreq1, 'low'); % Butterworth low-pass filter coefficients
                    filteredNoise = filter(b1, a1, noise_sensor); % Apply the low-pass filter to the white noise
                end
                inputArg = inputArg + filteredNoise + Backgroundnoise_sensor;
            end
            outputArg = inputArg;
        end

        function outputArg = twoNoise(obj,inputArg)
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

            % ADDING NOISE TO MODIFIED SIGNAL (since they got filtered)
            % Define the parameters for white noise
            noiseAmplitude = 150; % Pa Amplitude of the white noise
            lowerAmplitude = 10; % Pa
            
            % Generate white noise
            noise2 = noiseAmplitude * randn(size(inputArg)); % Generate white noise with the same size as the sensor signal
            %noise3 = noiseAmplitude * randn(size(inputArg)); % Generate white noise with the same size as the sensor signal
            Backgroundnoise2 = lowerAmplitude * randn(size(inputArg)); % Generate white noise with the same size as the sensor signal
            %Backgroundnoise3 = lowerAmplitude * randn(size(inputArg)); % Generate white noise with the same size as the sensor signal
            
            
            Fs = 50; %Hz Change this to the appropriate value
            % Apply low-pass filtering to the white noise
            cutOffFrequency1 = 1; % Cut-off frequency of the low-pass filter for ascension
            %cutOffFrequency2 = 3; % Cut-off frequency of the low-pass filter for desention
            normalizedCutOffFrequency1 = cutOffFrequency1 / (0.5 * Fs); % Normalize the cut-off frequency
            %normalizedCutOffFrequency2 = cutOffFrequency2 / (0.5 * Fs); % Normalize the cut-off frequency
            [b1, a1] = butter(4, normalizedCutOffFrequency1, 'low'); % Butterworth low-pass filter coefficients
            %[b2, a2] = butter(4, normalizedCutOffFrequency2, 'low'); % Butterworth low-pass filter coefficients
            % filteredNoise2 = zeros(length(noise2),1);
            % filteredNoise3 = zeros(length(noise3),1);
            filteredNoise2(1:1087) = filter(b1, a1, noise2(1:1087)); % Apply the low-pass filter to the white noise
            
            
            %filteredNoise3(1087:12640) = filter(b2, a2, noise3(1087:end)); % Apply the low-pass filter to the white noise
            
            
            % Add white noise to the sensor signal
            outputArg = inputArg + filteredNoise2 + Backgroundnoise2;
            %pyxis_flight_roccaraso.sensor3 = pyxis_flight_roccaraso.sensor3 + filteredNoise3 + Backgroundnoise3;

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

