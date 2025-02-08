classdef Sensor1D < handle

    % Author: Stefano Belletti, Samuel Flore
    % Skyward Experimental Rocketry | AVN - GNC
    % email: stefano.belletti@skywarder.eu
    % Release date: 18/11/2024
    % 
    % Sensor class for 1D sensors, adding properties and noise as defined
    % in initSensorsYYYY_Mission.m
    % 
    % Creating a new sensor: [obj] = Sensor1D()
    
    properties
        minMeasurementRange;                    % Max limit of sensor
        maxMeasurementRange;                    % Min limit of sensor
        bit;                                    % Number of bits for the sensor (if available)
        resolution;                             % resolution of the sensor (set internally if bit is available)
        dt;                                     % Sampling time
        
        % noises
        noiseType;                              % White (default) or Pink
        noiseDataTrack1;                        % Noise data for track1 (only data for 1D sensors)
        noiseFactor;                            % Scale factor
        colored_opts;                           % Colored data noise

        noiseVariance;                          % Defining gaussian white noise
        
        % offset
        offset;                                 % Offset in all directions
        tempOffset;                             % Coefficient for temperature depending offset
        error2dOffset;                          % first column: inputArg, second column: relativeArg, third column: error
    end
    
    methods (Access = 'public')
        function obj = Sensor1D()
            % creating a new sensor
        end

        function [outputArg] = sens(obj,inputArg,temp,t)
            % Sens loop: here the ideal input is transformed into the real
            % output
            inputArg = obj.addOffset(inputArg);
            inputArg = obj.add2DOffset(inputArg,temp);
            inputArg = obj.addTempOffset(inputArg,temp);
            inputArg = obj.addNoise(inputArg,t);
            inputArg = obj.quantization(inputArg);
            inputArg = obj.saturation(inputArg);
            outputArg = inputArg;
        end

        function [] = update(obj,varargin)
            if isa(obj,'SensorGPS')
                % Noise is still unknown
                error("Do not use .update for class SensorGPS")
            end

            % Check for crucial data
            if isempty(obj.minMeasurementRange)
                error("Empty minMeasurementRange")
            end
            if isempty(obj.maxMeasurementRange)
                error("Empty minMeasurementRange")
            end

            if isempty(obj.resolution)
                if ~isempty(obj.bit)
                    obj.resolution = (obj.maxMeasurementRange - obj.minMeasurementRange)/(2^obj.bit);
                else
                    error("Empty bit")
                end
            end

            % Noise initialization
            if size(varargin,2) == 3
                % Find the correct element of the vector
                vect = varargin{1};
                name = varargin{2};
                number = varargin{3};

                len = length(vect);

                for ii = 1:len
                    found = strcmp(name, vect(ii).name);
                    if found
                        if number>1
                            number = number - 1;
                        else
                            break
                        end
                    end
                end
                
                if found
                    obj.noiseType = vect(ii).noise_type;
                    
                    if strcmp("Sensor1D", class(obj)) || strcmp("SensorFault", class(obj))
                        obj.noiseDataTrack1 = vect(ii).track1;
                        obj.noiseFactor = vect(ii).factor;
                    elseif strcmp("Sensor3D", class(obj)) || strcmp("SensorGPS", class(obj))
                        obj.noiseDataTrack1 = vect(ii).track1;
                        obj.noiseDataTrack2 = vect(ii).track2;
                        obj.noiseDataTrack3 = vect(ii).track3;
                        obj.noiseFactor = vect(ii).factor;
                    else
                        error("Sensor not defined")
                    end

                    if strcmp(obj.noiseType, "colored") || strcmp(obj.noiseType, "pink")
                        obj.colored_opts.white_variance = vect(ii).colored_data.white_variance;
                        obj.colored_opts.fcut = vect(ii).colored_data.fcut;
                        obj.colored_opts.butterOrder = vect(ii).colored_data.butterOrder;
                        % "Filter's memory":
                        obj.colored_opts.filterStatus1 = 0;
                        obj.colored_opts.filterStatus2 = 0;
                        obj.colored_opts.filterStatus3 = 0;
                    end
                else
                    % if strcmp("Sensor1D", class(obj)) || strcmp("SensorFault", class(obj))
                    %     obj.noiseDataTrack1 = [];
                    % elseif strcmp("Sensor3D", class(obj)) || strcmp("SensorGPS", class(obj))
                    %     obj.noiseDataTrack1 = [];
                    %     obj.noiseDataTrack2 = [];
                    %     obj.noiseDataTrack3 = [];
                    % else
                    %     error("Sensor not defined")
                    % end
                    error("Sensor not found")
                end
            end
        end
    end

    methods (Access = 'protected')
        function outputArg = addOffset(obj,inputArg)
            % Adding offset to a 2D sensor
            if (~isempty(obj.offset))
                inputArg=inputArg+ones(size(inputArg)).*obj.offset;
            end
            outputArg = inputArg;
        end

        function outputArg = add2DOffset(obj,inputArg,temp)
            % Adding 2DOffset if present
            if (~isempty(obj.error2dOffset))
                inputArg=inputArg + griddata(obj.error2dOffset(:,1),obj.error2dOffset(:,2),obj.error2dOffset(:,3),inputArg,temp);
            end
            outputArg = inputArg;
        end

        function outputArg = addTempOffset(obj,inputArg,temp)
            % Add temperature offset
            if (~isempty(obj.tempOffset))
                inputArg=inputArg+ones(size(inputArg)).*temp*obj.tempOffset;
            end
            outputArg = inputArg;
        end

        function outputArg = addNoise(obj,inputArg,t)
            % Add noise, depending on the type and magnitude
            if ~isempty(obj.noiseVariance) % white noise override
                inputArg = inputArg + sqrt(obj.noiseVariance).*randn(length(inputArg),1);
            elseif ~isempty(obj.noiseDataTrack1)
                if strcmp(obj.noiseType, "white")
                    inputArg = inputArg + sqrt(obj.noiseDataTrack1*obj.noiseFactor^2).*randn(length(inputArg),1);
                elseif strcmp(obj.noiseType, "pink")
                    for ii = 1:length(obj.noiseDataTrack1.peaks_vect_f)
                        inputArg = inputArg + obj.noiseDataTrack1.peaks_vect_val(ii)*obj.noiseFactor*sin(2*pi*obj.noiseDataTrack1.peaks_vect_f(ii)*t + randn(1));
                    end
                    % Colored noise
                    white_noise = sqrt(obj.noiseDataTrack1.variance*obj.noiseFactor^2).*randn(length(inputArg),1);
                    [b, a] = butter(obj.colored_opts.butterOrder, obj.colored_opts.fcut, 'low');
                    [colored_noise, obj.colored_opts.filterStatus1] = filter(b, a, white_noise, obj.colored_opts.filterStatus1);
                    inputArg = inputArg + colored_noise;
                elseif strcmp(obj.noiseType, "colored")
                    inputArg = inputArg + sqrt(obj.noiseDataTrack1.white_variance*obj.noiseFactor^2).*randn(length(inputArg),1);
                    [b, a] = butter(obj.noiseDataTrack1.butterOrder, obj.noiseDataTrack1.fcut, 'low');
                    [inputArg, obj.colored_opts.filterStatus1] = filter(b, a, inputArg, obj.colored_opts.filterStatus1);
                else
                    error("This noise is not defined")
                end
            end
            outputArg = inputArg;
        end

        function outputArg = quantization(obj,inputArg)
            % Quantization of the input
            if ~isempty(obj.resolution)
                inputArg = obj.resolution*round(inputArg/obj.resolution);
            end
            outputArg = inputArg;
        end

        function outputArg = saturation(obj,inputArg)
            % Add sensor saturation
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

