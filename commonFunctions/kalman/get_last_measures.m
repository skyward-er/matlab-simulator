function measures = get_last_measures(measures, sensorTot, t, settings)
%{  
%}

%% IMU
index_imu   =  sum(t >= sensorTot.imu.time);
if settings.second_imu && (settings.shutdown || ~settings.flagAscent)
    % Use the precision IMU when available not in power ascent
    measures.acc = sensorTot.imu_1.accelerometer_measures(index_imu, :);
    measures.gyro = sensorTot.imu_1.gyro_measures(index_imu, :);

else
    % Use the standard IMU if in power ascent or if the second IMU is not available
    measures.acc  = sensorTot.imu.accelerometer_measures(index_imu, :);
    measures.gyro = sensorTot.imu.gyro_measures(index_imu, :);
end

%% BAROMETER
index_bar   =  sum(t >= sensorTot.barometer.time);
measures.baro = sensorTot.barometer.pressure_measures(index_bar);

%% GPS
index_GPS   =  sum(t >= sensorTot.gps.time);
measures.gps.pos = sensorTot.gps.position_measures(index_GPS,1:2);
measures.gps.vel = sensorTot.gps.velocity_measures(index_GPS,1:2);
measures.gps.lastindex = measures.gps.index;
measures.gps.index = index_GPS;

%% PITOT
index_pit   =  sum(t >= sensorTot.pitot.time);
measures.pitot.dyn_press = sensorTot.pitot.dynamic_pressure(index_pit,:);
measures.pitot.stat_press = sensorTot.pitot.static_pressure(index_pit,:);
measures.pitot.mach = sensorTot.pitot.Mach(index_pit, :);
measures.pitot.lastindex = measures.pitot.index;
measures.pitot.index = index_pit;

end