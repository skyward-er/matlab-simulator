classdef Sensor2D < handle

    % Author: Stefano Belletti, Samuel Flore
    % Skyward Experimental Rocketry | AVN - GNC
    % email: stefano.belletti@skywarder.eu
    % Release date: 18/11/2024
    % 
    % Sensor class for 2D sensors
    % 
    % Creating a new sensor: [obj] = Sensor2D()
    
    properties
        minMeasurementRange;                    % Max limit of sensor
        maxMeasurementRange;                    % Min limit of sensor
        bit;                                    % Number of bits for the sensor (if available)
        resolution;                             % resolution of the sensor (set internally if bit is available)
        dt;                                     % Sampling time

        % noises
        noiseType;                              % White (default) or Pink
        noiseVariance;                          % Defining gaussian white noise
        pink_properties;                        % Defining pink noise
        
        % offset
        offset;                                 % Offset in all directions
        tempOffset;                             % Coefficient for temperature depending offset
        error2dOffset;                          % first column: inputArg, second column: relativeArg, third column: error
    end
    
    methods (Access = 'public')
        function obj = Sensor2D()
        % creating a new sensor
            obj.noiseType = "White";
        end

        function [outputArg] = sens(obj,inputArg,temp)
            inputArg = obj.addOffset(inputArg);
            inputArg = obj.add2DOffset(inputArg,temp);
            inputArg = obj.addTempOffset(inputArg,temp);
            inputArg = obj.addNoise(inputArg);
            inputArg = obj.quantization(inputArg);
            inputArg = obj.saturation(inputArg);
            outputArg = inputArg;
        end
    end

    methods (Access = 'protected')
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

        function outputArg = addNoise(obj,inputArg)
            if obj.noiseType == "White"
                inputArg=inputArg+sqrt(obj.noiseVariance).*randn(length(inputArg),1);
            elseif obj.noiseType == "Pink"
                % TBI
            end
            outputArg = inputArg;
        end

        function outputArg = quantization(obj,inputArg)
            if isempty(obj.resolution)
                if (~isempty(obj.maxMeasurementRange)) && (~isempty(obj.minMeasurementRange)) && (~isempty(obj.bit))
                    obj.resolution = (obj.maxMeasurementRange - obj.minMeasurementRange)/(2^obj.bit);
                end
            end
            if ~isempty(obj.resolution)
                inputArg = obj.resolution*round(inputArg/obj.resolution);
            end
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
    end
end

