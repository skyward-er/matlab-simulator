classdef GPS < Sensor3D
    
    % Author: Jan Hammelman
    % Skyward Experimental Rocketry | ELC-SCS Dept | electronics@skywarder.eu
    % email: jan.hammelmann@skywarder.eu,alessandro.delduca@skywarder.eu
    % Release date: 01/03/2021
    
    %   GPS Sensor
    %   Uses Sensor3D methods but tronsformes lat and lon degree data in
    %   meter before and afterwards transformes it back to degree
    
    methods
        function obj = GPS()
            %GPS Construct an instance of this class
            %   Calls the constructor of Sensor3D
            
            obj=obj@Sensor3D();
        end
        
        function [outPos, outVel] = sens(obj,state,temp,lat0,lon0)
            %METHOD1 Summary of this method goes here
            %   Fransforming lat and lat in meter and then call sens method
            %   of Sensor3D https://en.wikipedia.org/wiki/Geographic_coordinate_system
            %
            %  Inputs:
            %  state: real position and velocity vector in NED frame 
            %  temp: temperature of the sensor
            %  lat0: launchpad latitude;
            %  lon0: launchpad longitude;
            %
            %  Outputs:
            %  outLat,outLon,outAlt: sensor data with nois,
            %  offsets, etc.
            
            mLat= 1/111132.95225 * state(1) + lat0;
            mLon= 1  / (111412.87733*cosd(mLat)) * state(2) + lon0;
            
            inputArg = [mLat; mLon; state(3:end)]; 
            [outMeas] = sens@Sensor(obj,inputArg,temp);
            outPos = outMeas(1:3);
            outVel = outMeas(4:6);
        end
    end
end

