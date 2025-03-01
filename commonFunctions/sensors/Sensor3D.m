classdef Sensor3D < Sensor1D

    % Author: Stefano Belletti, Samuel Flore
    % Skyward Experimental Rocketry | AVN - GNC
    % email: stefano.belletti@skywarder.eu
    % Release date: 18/11/2024
    % 
    % Sensor class for 3D sensors, adding properties and noise as defined
    % in initSensorsYYYY_Mission.m
    % 
    % Creating a new sensor: [obj] = Sensor3D()
    
    properties
        noiseDataTrack2;                        % Noise data for track2
        noiseDataTrack3;                        % Noise data for track3
        
        offsetX;                                % Offset in X direction
        offsetY;                                % Offset in Y direction
        offsetZ;                                % Offset in Z direction

        walkDiffusionCoef;                      % Random walk coefficient
        transMatrix;                            % Alignment matrix
    end

    properties (Access = 'protected')
        % state walk = drift
        stateWalkX;                             % Walk in X direction
        stateWalkY;                             % Walk in Y direction
        stateWalkZ;                             % Walk in Z direction
    end
    
    methods (Access = 'public')
        function obj = Sensor3D()
            obj.stateWalkX = 0;                 % Initial X walk = 0
            obj.stateWalkY = 0;                 % Initial Y walk = 0
            obj.stateWalkZ = 0;                 % Initial Z walk = 0
        end

        function [outputArg1,outputArg2,outputArg3] = sens(obj,inputArg1,inputArg2,inputArg3,temp,t)
            % Sens loop: here the ideal input is transformed into the real
            % output
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

            [inputArg1,inputArg2,inputArg3] = obj.addNoise3D(inputArg1,inputArg2,inputArg3,t);

            inputArg1 = obj.quantization(inputArg1);
            inputArg2 = obj.quantization(inputArg2);
            inputArg3 = obj.quantization(inputArg3);

            inputArg1 = obj.saturation(inputArg1);
            inputArg2 = obj.saturation(inputArg2);
            inputArg3 = obj.saturation(inputArg3);

            outputArg1 = inputArg1;
            outputArg2 = inputArg2;
            outputArg3 = inputArg3;
        end
    end

    methods (Access = 'protected')
        function [outputArg1,outputArg2,outputArg3] = addOffset3D(obj,inputArg1,inputArg2,inputArg3)        
            % Adding 3D offset
            if (~isempty(obj.offsetX)) && obj.offsetX
                inputArg1 = inputArg1 + ones(size(inputArg1)).*obj.offsetX;
            end
            if (~isempty(obj.offsetY)) && obj.offsetY
                inputArg2 = inputArg2 + ones(size(inputArg2)).*obj.offsetY;
            end
            if (~isempty(obj.offsetZ)) && obj.offsetZ
                inputArg3 = inputArg3 + ones(size(inputArg3)).*obj.offsetZ;
            end
            outputArg1 = inputArg1;
            outputArg2 = inputArg2;
            outputArg3 = inputArg3;
        end

        function [outputArgX,outputArgY,outputArgZ] = randomWalk(obj,inputArgX,inputArgY,inputArgZ)
            % Adding random walk
            if (~isempty(obj.walkDiffusionCoef)) && obj.walkDiffusionCoef
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
            
            outputArgX=inputArgX+obj.stateWalkX;
            outputArgY=inputArgY+obj.stateWalkY;
            outputArgZ=inputArgZ+obj.stateWalkZ;
        end

        function [outputArg1,outputArg2,outputArg3] = tranformAxis(obj,inputArg1,inputArg2,inputArg3)
            % Adding transformation
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

        function [outputArg1,outputArg2,outputArg3] = addNoise3D(obj,inputArg1,inputArg2,inputArg3,t)
            % Check for all three channels
            if ~isempty(obj.noiseDataTrack1) + ~isempty(obj.noiseDataTrack2) + ~isempty(obj.noiseDataTrack3) ~= 3
                error("The 3D sensor does not have 3 noise channels")
            end

            % Adding noise
            if ~isempty(obj.noiseVariance) % white noise override
                inputArg1 = inputArg1 + sqrt(obj.noiseVariance).*randn(length(inputArg1),1);
                inputArg2 = inputArg2 + sqrt(obj.noiseVariance).*randn(length(inputArg2),1);
                inputArg3 = inputArg3 + sqrt(obj.noiseVariance).*randn(length(inputArg3),1);
            elseif ~isempty(obj.noiseDataTrack1) && ~isempty(obj.noiseDataTrack2) && ~isempty(obj.noiseDataTrack3)
                if strcmp(obj.noiseType, "white")
                    inputArg1 = inputArg1 + sqrt(obj.noiseDataTrack1*obj.noiseFactor^2).*randn(length(inputArg1),1);
                    inputArg2 = inputArg2 + sqrt(obj.noiseDataTrack2*obj.noiseFactor^2).*randn(length(inputArg2),1);
                    inputArg3 = inputArg3 + sqrt(obj.noiseDataTrack3*obj.noiseFactor^2).*randn(length(inputArg3),1);
                elseif strcmp(obj.noiseType, "pink")
                    for ii = 1:length(obj.noiseDataTrack1.peaks_vect_f)
                        inputArg1 = inputArg1 + obj.noiseDataTrack1.peaks_vect_val(ii)*obj.noiseFactor*sin(2*pi*obj.noiseDataTrack1.peaks_vect_f(ii)*t + randn(1));
                        inputArg2 = inputArg2 + obj.noiseDataTrack2.peaks_vect_val(ii)*obj.noiseFactor*sin(2*pi*obj.noiseDataTrack2.peaks_vect_f(ii)*t + randn(1));
                        inputArg3 = inputArg3 + obj.noiseDataTrack3.peaks_vect_val(ii)*obj.noiseFactor*sin(2*pi*obj.noiseDataTrack3.peaks_vect_f(ii)*t + randn(1));
                    end
                else
                    error("This noise is not defined")
                end

                if strcmp(obj.noiseType, "pink") || strcmp(obj.noiseType, "colored")
                    % Colored noise: added to pink and colored noise types
                    white_noise1 = sqrt(obj.noiseDataTrack1.variance*obj.noiseFactor^2).*randn(1,1);
                    [b1, a1] = butter(obj.noiseDataTrack1.butterOrder, obj.noiseDataTrack1.fcut, 'low');
                    [colored_noise1, obj.colored_opts.filterStatus1] = filter(b1, a1, white_noise1, obj.colored_opts.filterStatus1);
                    white_noise2 = sqrt(obj.noiseDataTrack2.variance*obj.noiseFactor^2).*randn(1,1);
                    [b2, a2] = butter(obj.noiseDataTrack2.butterOrder, obj.noiseDataTrack2.fcut, 'low');
                    [colored_noise2, obj.colored_opts.filterStatus2] = filter(b2, a2, white_noise2, obj.colored_opts.filterStatus2);
                    white_noise3 = sqrt(obj.noiseDataTrack3.variance*obj.noiseFactor^2).*randn(1,1);
                    [b3, a3] = butter(obj.noiseDataTrack3.butterOrder, obj.noiseDataTrack3.fcut, 'low');
                    [colored_noise3, obj.colored_opts.filterStatus3] = filter(b3, a3, white_noise3, obj.colored_opts.filterStatus3);

                    inputArg1 = inputArg1 + colored_noise1;
                    inputArg2 = inputArg2 + colored_noise2;
                    inputArg3 = inputArg3 + colored_noise3;
                end
            end

            outputArg1 = inputArg1;
            outputArg2 = inputArg2;
            outputArg3 = inputArg3;
        end
    end
end

