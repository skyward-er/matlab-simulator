classdef SensorGPS < Sensor3D
    
    % Author: Stefano Belletti, Samuel Flore
    % Skyward Experimental Rocketry | AVN - GNC
    % email: stefano.belletti@skywarder.eu
    % Release date: 18/11/2024
    % 
    % Sensor class for GPS
    % 
    % Creating a new sensor: [obj] = GPS()
    
    % GPS Sensor
    % Uses Sensor3D methods but transforms lat and lon degree data in
    % meter before and afterwards transforms it back to degree
    
    methods
        function obj = SensorGPS()
            obj=obj@Sensor3D();
        end
        
        function [outPos, outVel] = sens(obj,state,temp,lat0,lon0,t)
            mLat= 1/111132.95225 * state(1) + lat0;
            mLon= 1  / (111412.87733*cosd(mLat)) * state(2) + lon0;
            
            inputArg = [mLat; mLon; state(3:end)]; 
            [outMeas] = sens@Sensor1D(obj,inputArg,temp,t);
            outPos = outMeas(1:3);
            outVel = outMeas(4:6);
        end
    end
end

