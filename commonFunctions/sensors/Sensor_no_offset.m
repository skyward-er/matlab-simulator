classdef Sensor_no_offset < handle
    
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
         
        dt; % Sampling time
       
    end
    
    
    methods (Access='public')
        function obj = Sensor_no_offset()
            %SENSOR Construct an instance of this class
            %   Sensor is the Superclass of all sensors
            
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
            
            inputArg=obj.whiteNoise(inputArg);
            inputArg=obj.addOffset(inputArg);
            inputArg=obj.quantization(inputArg);
            inputArg=obj.saturation(inputArg); 
            outputArg = inputArg;
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
            
            if (~isempty(obj.noiseVariance))
%                 inputArg=inputArg+ones(size(inputArg)).*sqrt(obj.noiseVariance).*randn(1,1);,
                inputArg=inputArg+sqrt(obj.noiseVariance).*randn(size(inputArg));
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
        
    end
end

