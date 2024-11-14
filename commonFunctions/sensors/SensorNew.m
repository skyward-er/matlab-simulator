classdef SensorNew < dynamicprops
    % properties
    properties (Access='public')
        % phisical characteristics
        minMeasurementRange;                    % Max limit of sensor
        maxMeasurementRange;                    % Min limit of sensor
        bit;                                    % number of bits for the sensor ( if available)
        resolution;                             % resolution of the sensor
        dt;                                     % Sampling time
        
        % noise
        noiseVariance;                          % Varianze for the gaussian white noise
        
        % offset
        offset;                                 % Offset in all directions
        tempOffset;                             % Coefficent for temperature depending offset
        error2dOffset;                          % first column: inputArg, second column: relativeArg, third column: error
        
        % fault
        fault_time;                             % if set to -1 it will be randomly chosen between a min and a max time in seconds
        min_fault_time;                         % lower bound of the time window where the fault can occur in case it's randomly chosen
        max_fault_time;                         % upper bound of the time window where the fault can occur in case it's randomly chosen
    end
    
    % methods
    methods (Access='public')
        function obj = SensorNew(is3D)
            % creating a new sensor

            % adding Sensor3D properties if is3D == true
            if is3D
                obj.addprop("is3D");
                obj.is3D = true;

                obj.addprop("TD_offset");
                obj.addprop("TD_walkDiffusionCoef");
                obj.addprop("TD_transMatrix");

                obj.addprop("TD_stateWalk");
                obj.TD_stateWalk.x = 0;
                obj.TD_stateWalk.y = 0;
                obj.TD_stateWalk.z = 0;
            else
                obj.addprop("is3D");
                obj.is3D = false;
            end
        end
        
        function outputArg = sens(obj,inputArg,temp)
            if ~isstruct(inputArg)
                inputArg = obj.addOffset(inputArg);
                inputArg = obj.add2DOffset(inputArg,temp);
                inputArg = obj.addTempOffset(inputArg,temp);
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

    methods (Access='protected')
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

        % old 2D
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

        function outputArg = quantization(obj,inputArg)
            if (~isempty(obj.resolution))
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

