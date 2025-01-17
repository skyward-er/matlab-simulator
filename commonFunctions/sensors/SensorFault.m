classdef SensorFault < Sensor1D
    
    % Author: Stefano Belletti, Samuel Flore
    % Skyward Experimental Rocketry | AVN - GNC
    % email: stefano.belletti@skywarder.eu
    % Release date: 18/11/2024
    % 
    % Sensor class for SensorFault
    % 
    % Creating a new sensor: [obj] = Sensor1D()

    properties (Access='public')
        fault_time; % if set to -1 it will be randomly chosen between a min and a max time in seconds

        max_fault_time; %upper bound of the time window where the fault can occur in case it's randomly chosen

        min_fault_time; %lower bound of the time window where the fault can occur in case it's randomly chosen

        % attributes used for fault simulation
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
end

