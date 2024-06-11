function nas_result = simulateNAS(sim_output, settings, algorithm)
%% simulateNAS: Run NAS against the simulation output
%
% Inputs:
% - sim_output: the output of the simulation
% - settings: the settings of the simulation
% - algorithm: the algorithm to run. Can be one of the following:
%   - "acc": only acceleration prediction
%   - "gyro": only gyroscope prediction
%   - "gps": only GPS correction
%   - "baro": only barometer correction
%   - "pitot": only pitot correction
%   - "mag": only magnetometer correction
%   - "complete": all algorithms
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
% - steps: Array of the different prediction made during every iteration.

    algorithms = ["acc", "gyro", "gps", "baro", "pitot", "mag", "complete"];
    if ~ismember(algorithm, algorithms)
        error('Invalid algorithm. Must be one of: %s', strjoin(algorithms, ', '));
    end

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

    acc_data    = single(zeros(length(t_nas)-1, size(sim_acc_data   , 2)    ));
    gyro_data   = single(zeros(length(t_nas)-1, size(sim_gyro_data  , 2)    ));
    mag_data    = single(zeros(length(t_nas)-1, size(sim_mag_data   , 2)    ));
    baro_data   = single(zeros(length(t_nas)-1, size(sim_baro_data  , 2)    ));
    pitot_data  = single(zeros(length(t_nas)-1, size(sim_pitot_data , 2)    ));
    gps_data    = single(zeros(length(t_nas)-1, size(sim_gps_data   , 2) + 5));

    input_data  = cell(length(t_nas)-1, 3);
    output_data = cell(length(t_nas)-1, 3);

    disp(newline + "Running NAS...");

    for idx = 2:length(t_nas)
        dt_k      = t_nas(idx) - t_nas(idx-1);
        timestamp = floor(t_nas(idx) * 1e6); % Timestamp in microseconds, for boardcore
        x_lin     = state_in(1:6);
        x_quat    = state_in(7:13);
        P_lin     = P_in(1:6, 1:6);
        P_quat    = P_in(7:12, 7:12);

        input_data{idx, 1} = timestamp;
        input_data{idx, 2} = state_in;
        input_data{idx, 3} = P_in;

        % Predict acc
        index_acc = sum(t_nas(idx) >= sim_acc_data(:,1));
        acc_data(idx, :) = [timestamp sim_acc_data(index_acc, 2:end)];
        
        if ismember(algorithm, ["acc", "complete"])
            [x_lin, ~, P_lin] = predictorLinear2(x_lin, P_lin, dt_k, sim_acc_data(index_acc, 2:end), x_quat(1:4), nas_struct.QLinear);
        end
        acc_lin = x_lin;

        % Predict gyro
        index_gyro = sum(t_nas(idx) >= sim_gyro_data(:,1));            
        gyro_data(idx, :) = [timestamp sim_gyro_data(index_gyro, 2:end)];
        
        if ismember(algorithm, ["gyro", "complete"])
            [x_quat, P_quat] = predictorQuat(x_quat, P_quat, sim_gyro_data(index_gyro, 2:end), dt_k, nas_struct.Qq);
        end
        gyro_quat = x_quat;

        % Correct gps
        index_gps = sum(t_nas(idx) >= sim_gps_data(:,1));
        [gps_coord(1), gps_coord(2), gps_coord(3)] = ned2geodetic(sim_gps_data(index_gps, 2), sim_gps_data(index_gps, 3), sim_gps_data(index_gps, 4), settings.lat0, settings.lon0, settings.z0, wgs84Ellipsoid);
        gps_data(idx, :) = [timestamp [gps_coord sim_gps_data(index_gps, 5:end) 0 0 0 16 3]];
        
        if norm(sim_acc_data(index_acc,2:end)) < 34 % around 3.5g
            if ismember(algorithm, ["gps", "complete"])
                [x_lin, P_lin, ~] = correctionGPS(x_lin, ...
                    P_lin, sim_gps_data(index_gps, 2:3), ...
                    sim_gps_data(index_gps, 5:6), nas_struct.sigma_GPS, ...
                    16, settings.lat0, settings.lon0, nas_struct.GPS.a, nas_struct.GPS.b);
            end
        end
        gps_lin = x_lin;


        % Correct barometer
        index_baro = sum(t_nas(idx) >= sim_baro_data(:,1));
        baro_data(idx, :) = [timestamp sim_pressure_data(index_baro, 2)];
        
        if ismember(algorithm, ["baro", "complete"])
            [x_lin, P_lin, ~] = correctionBarometer(x_lin, P_lin, sim_baro_data(index_baro, 2), nas_struct.sigma_baro, nas_struct.baro);
        end
        baro_lin = x_lin;

        % Correct magnetometer
        index_mag = sum(t_nas(idx) >= sim_mag_data(:,1));
        mag_data(idx, :) = [timestamp sim_mag_data(index_mag, 2:end)];
        
        if ismember(algorithm, ["mag", "complete"])
            [x_quat, P_quat, ~, ~] = correctorQuat(x_quat, P_quat, sim_mag_data(index_mag, 2:end), nas_struct.sigma_mag, XYZ0*0.01);
        end
        mag_quat = x_quat;

        % Correct Pitot
        index_pitot = sum(t_nas(idx) >= sim_pitot_data(:,1));
        pitot_data(idx, :) = [timestamp sim_pitot_data(index_pitot, 2)];
        
        if ismember(algorithm, ["pitot", "complete"])
            [x_lin, P_lin(4:6,4:6), ~] = correctionPitot_airspeed(x_lin, P_lin(4:6,4:6), sim_pitot_data(index_pitot, 2:end), nas_struct.sigma_pitot2, settings.OMEGA);
        end
        pitot_lin = x_lin;

        output_data{idx, 1} = timestamp;
        output_data{idx, 2} = [x_lin x_quat];
        output_data{idx, 3} = blkdiag(P_lin, P_quat);

        steps(idx) = struct(        ...
            'acc_lin',   acc_lin,   ...
            'gyro_quat', gyro_quat, ...
            'gps_lin',   gps_lin,   ...
            'baro_lin',  baro_lin,  ...
            'mag_quat',  mag_quat,  ...
            'pitot_lin', pitot_lin  ...
        );

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
        'gps',      gps_data,        ...
        'steps',    steps            ...
    );
end