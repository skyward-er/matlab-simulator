function nas_result = simulateNAS(sim_output, settings)
%% simulateNAS: Run NAS against the simulation output
%
% Inputs:
% - sim_output: the output of the simulation
% - settings: the settings of the simulation
%
% Outputs:
%
% Struct with the following fields:
% - input: the input data for the NAS
% - output: the output data of the NAS
% - acc: accelerometer data
% - gyro: gyroscope data
% - mag: magnetometer data
% - baro: barometer data
% - pitot: pitot data
% - gps: GPS data

    nas_struct = settings.nas;

    %% Initial state configuration
    % Attitude
    Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180, 'ZYX')';
    % State
    X0 = [0; 0; -settings.z0]; % Position initial condition
    V0 = [0; 0; 0];            % Velocity initial condition
    nasData  = [X0; V0; Q0(2:4); Q0(1); 0; 0; 0]';
    state_in = single(nasData);
    P_in     = single(0.01*eye(12));
    std_magneticField; % Regenerate magnetic field data

    %% Raw sensor data from the simulation
    sim_acc_data      = single([sim_output.sensors.imu.time       sim_output.sensors.imu.accelerometer_measures   ]);
    sim_gyro_data     = single([sim_output.sensors.imu.time       sim_output.sensors.imu.gyro_measures            ]);
    sim_mag_data      = single([sim_output.sensors.imu.time       sim_output.sensors.imu.magnetometer_measures    ]);
    sim_baro_data     = single([sim_output.sensors.barometer.time sim_output.sensors.barometer.altitude           ]);
    sim_pressure_data = single([sim_output.sensors.barometer.time sim_output.sensors.barometer.pressure_measures  ]);
    sim_pitot_data    = single([sim_output.sensors.pitot.time     sim_output.sensors.pitot.airspeed               ]);
    sim_gps_data      = single([sim_output.sensors.gps.time       sim_output.sensors.gps.position_measures      ...   
                                sim_output.sensors.gps.velocity_measures]);
    
    %% Actual data used by the NAS
    t_nas = single(sim_output.sensors.nas.time);

    acc_data    = single(zeros(length(t_nas)-1, size(sim_acc_data   , 2)-1    ));
    gyro_data   = single(zeros(length(t_nas)-1, size(sim_gyro_data  , 2)-1    ));
    mag_data    = single(zeros(length(t_nas)-1, size(sim_mag_data   , 2)-1    ));
    baro_data   = single(zeros(length(t_nas)-1, size(sim_baro_data  , 2)-1    ));
    pitot_data  = single(zeros(length(t_nas)-1, size(sim_pitot_data , 2)-1    ));
    gps_data    = single(zeros(length(t_nas)-1, size(sim_gps_data   , 2)-1 + 5));

    input_data  = cell(length(t_nas)-1, 3);
    output_data = cell(length(t_nas)-1, 3);

    disp('Running NAS...');

    for idx = 2:length(t_nas)
        dt_k   = t_nas(idx) - t_nas(idx-1);
        x_lin  = state_in(1:6);
        x_quat = state_in(7:13);
        P_lin  = P_in(1:6, 1:6);
        P_quat = P_in(7:12, 7:12);

        input_data{idx, 1} = idx;
        input_data{idx, 2} = state_in;
        input_data{idx, 3} = P_in;

        % Predict acc
        index_imu = sum(t_nas(idx) >= sim_acc_data(:,1));            
        acc_data(idx, :) = sim_acc_data(index_imu, 2:end);
        
        [x_lin, ~, P_lin] = predictorLinear2(x_lin, P_lin, dt_k, sim_acc_data(index_imu, 2:end), x_quat(1:4), nas_struct.QLinear);

        % Predict gyro
        index_imu = sum(t_nas(idx) >= sim_gyro_data(:,1));            
        gyro_data(idx, :) = sim_gyro_data(index_imu, 2:end);
        
        [x_quat, P_quat] = predictorQuat(x_quat, P_quat, sim_gyro_data(index_imu, 2:end), dt_k, nas_struct.Qq);
        
        % Correct gps
        index_gps = sum(t_nas(idx) >= sim_gps_data(:,1));
        [gps_coord(1), gps_coord(2), gps_coord(3)] = ned2geodetic(sim_gps_data(index_gps, 2), sim_gps_data(index_gps, 3), sim_gps_data(index_gps, 4), settings.lat0, settings.lon0, settings.z0, wgs84Ellipsoid);
        gps_data(idx, :) = [gps_coord sim_gps_data(index_gps, 5:end) 0 0 0 16 3];
        
        [x_lin, P_lin, ~] = correctionGPS(x_lin, P_lin, sim_gps_data(index_gps, 2:3), sim_gps_data(index_gps, 5:6), nas_struct.sigma_GPS, 16, 1);

        % Correct barometer
        index_baro = sum(t_nas(idx) >= sim_baro_data(:,1));
        baro_data(idx, :) = sim_pressure_data(index_baro, 2);
        
        [x_lin, P_lin, ~] = correctionBarometer(x_lin, P_lin, sim_baro_data(index_baro, 2), nas_struct.sigma_baro);

        % Correct magnetometer
        index_imu = sum(t_nas(idx) >= sim_mag_data(:,1));
        mag_data(idx, :) = sim_mag_data(index_imu, 2:end);
        
        [x_quat, P_quat, ~, ~] = correctorQuat(x_quat, P_quat, sim_mag_data(index_imu, 2:end), nas_struct.sigma_mag, XYZ0*0.01);

        % Correct Pitot
        index_pitot = sum(t_nas(idx) >= sim_pitot_data(:,1));
        pitot_data(idx, :) = sim_pitot_data(index_pitot, 2);
        
        [x_lin, P_lin(4:6,4:6), ~] = correctionPitot_airspeed(x_lin, P_lin(4:6,4:6), sim_pitot_data(index_pitot, 2:end), nas_struct.sigma_pitot2, settings.OMEGA);


        output_data{idx, 1} = 0;
        output_data{idx, 2} = [x_lin x_quat];
        output_data{idx, 3} = blkdiag(P_lin, P_quat);

        state_in(1:6)    = x_lin;
        state_in(7:13)   = x_quat;
        P_in(1:6, 1:6)   = P_lin;
        P_in(7:12, 7:12) = P_quat;
    end

    nas_result = struct(             ...
        'input',    { input_data },  ...
        'output',   { output_data }, ...
        'acc',      acc_data,        ...
        'gyro',     gyro_data,       ...
        'mag',      mag_data,        ...
        'baro',     baro_data,       ...
        'pitot',    pitot_data,      ... 
        'gps',      gps_data         ...
    );
end