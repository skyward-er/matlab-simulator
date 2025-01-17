classdef Sensor3D < Sensor1D

    % Author: Stefano Belletti, Samuel Flore
    % Skyward Experimental Rocketry | AVN - GNC
    % email: stefano.belletti@skywarder.eu
    % Release date: 18/11/2024
    % 
    % Sensor class for 3D sensors
    % 
    % Creating a new sensor: [obj] = Sensor3D()
    
    properties
        noiseDataTrack2;
        noiseDataTrack3;
        
        offsetX;
        offsetY;
        offsetZ;

        walkDiffusionCoef;
        transMatrix;
    end

    properties (Access = 'protected')
        stateWalkX;
        stateWalkY;
        stateWalkZ;
    end
    
    methods (Access = 'public')
        function obj = Sensor3D()
            obj.stateWalkX = 0;
            obj.stateWalkY = 0;
            obj.stateWalkZ = 0;
        end

        function [outputArg1,outputArg2,outputArg3] = sens(obj,inputArg1,inputArg2,inputArg3,temp,t)
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
            if ~isempty(obj.noiseVariance)              % check for old results
                inputArg1 = inputArg1 + sqrt(obj.noiseVariance).*randn(length(inputArg1),1);
                inputArg2 = inputArg2 + sqrt(obj.noiseVariance).*randn(length(inputArg2),1);
                inputArg3 = inputArg3 + sqrt(obj.noiseVariance).*randn(length(inputArg3),1);
            elseif ~isempty(obj.noiseDataTrack1)
                if strcmp(obj.noiseType, "white")
                    inputArg1 = inputArg1 + sqrt(obj.noiseDataTrack1*obj.noiseFactor).*randn(length(inputArg1),1);
                    inputArg2 = inputArg2 + sqrt(obj.noiseDataTrack2*obj.noiseFactor).*randn(length(inputArg2),1);
                    inputArg3 = inputArg3 + sqrt(obj.noiseDataTrack3*obj.noiseFactor).*randn(length(inputArg3),1);
                elseif strcmp(obj.noiseType, "pink")
                    for ii = 1:length(obj.noiseDataTrack1.peaks_vect_f)
                        inputArg1 = inputArg1 + obj.noiseDataTrack1.peaks_vect_val(ii)*obj.noiseFactor*sin(2*pi*obj.noiseDataTrack1.peaks_vect_f(ii)*t + randn(1));
                        inputArg2 = inputArg2 + obj.noiseDataTrack2.peaks_vect_val(ii)*obj.noiseFactor*sin(2*pi*obj.noiseDataTrack2.peaks_vect_f(ii)*t + randn(1));
                        inputArg3 = inputArg3 + obj.noiseDataTrack3.peaks_vect_val(ii)*obj.noiseFactor*sin(2*pi*obj.noiseDataTrack3.peaks_vect_f(ii)*t + randn(1));
                    end
                    inputArg1 = inputArg1 + sqrt(obj.noiseDataTrack1.variance*obj.noiseFactor).*randn(length(inputArg1),1);
                    inputArg2 = inputArg2 + sqrt(obj.noiseDataTrack2.variance*obj.noiseFactor).*randn(length(inputArg2),1);
                    inputArg3 = inputArg3 + sqrt(obj.noiseDataTrack3.variance*obj.noiseFactor).*randn(length(inputArg3),1);
                elseif strcmp(obj.noiseType, "colored")
                    error("Yet to be implemented")
                else
                    error("This noise is not defined")
                end
            end

            outputArg1 = inputArg1;
            outputArg2 = inputArg2;
            outputArg3 = inputArg3;
        end
    end
end

