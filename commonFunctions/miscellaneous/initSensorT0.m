function [t0] = initSensorT0(control,sensor)
%{ 
    Function used to initialize the value which indicates: 

    - the last time instant in which the sensor acquired some data if the
      frequency of the sensor is less than the one of the control 
    - the first time instant in which the sensor need to take a new
      mesurament
    
    INPUT:
    - control: frequency of the controller
    - sensor: frequency of the sensor
    
    OUTPUT: 
    - t0: the correct time instant
    
    NOTE:
    - this function is called in std_setInitialParams

    AUTHORS:
    - Giuseppe Brentino giuseppe.brentino@skywarder.eu
%}

if sensor<control
    t0 = -1/sensor;
else
    t0 = 0;
end

end

