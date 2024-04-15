function generate_NAS_catch_file(simOutput, settings, ConDataPath)
    if strcmpi(settings.scenario, 'descent')
        error("Catch test data cannot be used in descent case!");
    end
    
    folder = "HIL_CPP_files_NAS";
    if ~exist(ConDataPath+"/"+folder,"dir")
        mkdir(ConDataPath+"/"+folder)
    end

    %% configuration parameters

    std_magneticField; % Regenerate magnetic field data
    nas_struct = settings.nas;

    configNAS_export_table = table;
    configValues = [1/settings.frequencies.NASFrequency,settings.nas.sigma_beta,settings.nas.sigma_w,settings.nas.sigma_mag,settings.nas.sigma_GPS,settings.nas.sigma_baro, settings.nas.QLinear(1,1), settings.nas.QLinear(end, end), settings.nas.sigma_pitot2, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, single(XYZ0*0.01)', nas_struct.stopPitotAltitude];
    configNASvarNames = {'NAS_T', 'SIGMA_BETA','SIGMA_W','SIGMA_MAG','SIGMA_GPS', 'SIGMA_BAR', 'SIGMA_POS', 'SIGMA_VEL', 'SIGMA_PITOT', 'P_POS', 'P_POS_VERTICAL', 'P_VEL', 'P_VEL_VERTICAL', 'P_ATT', 'P_BIAS', 'MAG_NED x', 'MAG_NED y', 'MAG_NED z', 'PITOT_STOP_ALTITUDE'};
    for i = 1:size(configValues,2)
        configNAS_export_table(1,i) = table(configValues(1,i));
    end
    configNAS_export_table.Properties.VariableNames = configNASvarNames;
    writetable(configNAS_export_table,ConDataPath+"/"+folder+"/NAS_configNAS_"+settings.mission+".csv")

    %% NAS Algorithms rerun

    % Cases: "predict_acc", "predict_gyro", "correct_gps", "correct_baro",
    % "correct_mag", "correct_pitot", "complete" 
    algorithms = {'complete'}; % If left blank default is complete run of nas algorithm

    %% Initial state configuration
    % Attitude
    Q0 = angle2quat(settings.PHI, settings.OMEGA, 0*pi/180, 'ZYX')';
    % State
    X0 = [0; 0; -settings.z0];         % Position initial condition
    V0 = [0; 0; 0];         % Velocity initial condition
    nasData = [X0; V0; Q0(2:4); Q0(1);0;0;0]';
    state_in = single(nasData);
    P_in = single(0.01*eye(12));

    % Add gravity acceleration only when still on ramp
    if ~flagFlight
        simOutput.sensors.barometer.pressure_measures = simOutput.sensors.barometer.pressure_measures + (quat2dcm(Yf(:,10:13)) * [0;0;-9.81])';
    end

    %% Generate input files

    acc_data = single([simOutput.sensors.imu.time simOutput.sensors.imu.accelerometer_measures]);
    gyro_data = single([simOutput.sensors.imu.time simOutput.sensors.imu.gyro_measures]);
    mag_data = single([simOutput.sensors.imu.time simOutput.sensors.imu.magnetometer_measures]);
    baro_data = single([simOutput.sensors.barometer.time simOutput.sensors.barometer.altitude]);
    pressure_data = single([simOutput.sensors.barometer.time simOutput.sensors.barometer.pressure_measures]);
    gps_data = single([simOutput.sensors.gps.time simOutput.sensors.gps.position_measures simOutput.sensors.gps.velocity_measures]);
    pitot_data = single([simOutput.sensors.pitot.time simOutput.sensors.pitot.airspeed]);

    t_nas = single(simOutput.sensors.nas.time);
    input_data = cell(length(t_nas)-1, 9);
    output_data = cell(length(t_nas)-1, 3);

    Input_StateData_File = ConDataPath+"/"+folder+"/NAS_Input_StateData_"+settings.mission+".csv";
    AccData_File = ConDataPath+"/"+folder+"/NAS_AccData_"+settings.mission+".csv";
    GyroData_File = ConDataPath+"/"+folder+"/NAS_GyroData_"+settings.mission+".csv";
    GPSData_File = ConDataPath+"/"+folder+"/NAS_GPSData_"+settings.mission+".csv";
    BaroData_File = ConDataPath+"/"+folder+"/NAS_BaroData_"+settings.mission+".csv";
    MagData_File = ConDataPath+"/"+folder+"/NAS_MagData_"+settings.mission+".csv";
    PitotData_File = ConDataPath+"/"+folder+"/NAS_PitotData_"+settings.mission+".csv";
    Output_StateData_File = ConDataPath+"/"+folder+"/NAS_Output_StateData_"+settings.mission+".csv";

    %% Run NAS functions

    for ii = 2:length(t_nas)

        dt_k = t_nas(ii) - t_nas(ii-1);
        x_lin = state_in(1:6);
        x_quat = state_in(7:13);
        P_lin = P_in(1:6, 1:6);
        P_quat = P_in(7:12, 7:12);

        input_data{ii, 1} = ii;
        input_data{ii, 2} = state_in;
        input_data{ii, 3} = P_in;

        % Predict acc
        if isempty(algorithms) || any(ismember(algorithms, {'predict_acc', 'complete'}))
            index_imu = sum(t_nas(ii) >= acc_data(:,1));            
            [x_lin, ~, P_lin] = predictorLinear2(x_lin, P_lin, dt_k, acc_data(index_imu, 2:end), x_quat(1:4), nas_struct.QLinear);
            input_data{ii, 4} = acc_data(index_imu, 2:end);
        end

        % Predict gyro
        if isempty(algorithms) || any(ismember(algorithms, {'predict_gyro', 'complete'}))
            index_imu = sum(t_nas(ii) >= gyro_data(:,1));            
            [x_quat, P_quat] = predictorQuat(x_quat, P_quat, gyro_data(index_imu, 2:end), dt_k, nas_struct.Qq);
            input_data{ii, 5} = gyro_data(index_imu, 2:end);
        end
        
        % Correct gps
        if isempty(algorithms) || any(ismember(algorithms, {'correct_gps', 'complete'}))
            index_gps = sum(t_nas(ii) >= gps_data(:,1));
            [x_lin, P_lin, ~] = correctionGPS(x_lin, P_lin, gps_data(index_gps, 2:3), gps_data(index_gps, 5:6), nas_struct.sigma_GPS, 16, 1);
            [gps_coord(1), gps_coord(2), gps_coord(3)] = ned2geodetic(gps_data(index_gps, 2), gps_data(index_gps, 3), gps_data(index_gps, 4), settings.lat0, settings.lon0, settings.z0, wgs84Ellipsoid);
            input_data{ii, 6} = [gps_coord gps_data(index_gps, 5:end) 0 0 0 16 3];
        end

        % Correct barometer
        if isempty(algorithms) || any(ismember(algorithms, {'correct_baro', 'complete'}))
            index_baro = sum(t_nas(ii) >= baro_data(:,1));
            [x_lin, P_lin, ~] = correctionBarometer(x_lin, P_lin, baro_data(index_baro, 2), nas_struct.sigma_baro);
            input_data{ii, 7} = pressure_data(index_baro, 2);
        end

        % Correct magnetometer
        if isempty(algorithms) || any(ismember(algorithms, {'correct_mag', 'complete'}))
            index_imu = sum(t_nas(ii) >= mag_data(:,1));
            [x_quat, P_quat, ~, ~] = correctorQuat(x_quat, P_quat, mag_data(index_imu, 2:end), nas_struct.sigma_mag, XYZ0*0.01);
            input_data{ii, 8} = mag_data(index_imu, 2:end);
        end

        % Correct Pitot
        if isempty(algorithms) || any(ismember(algorithms, {'correct_pitot', 'complete'}))
            index_pitot = sum(t_nas(ii) >= pitot_data(:,1));
            [x_lin, P_lin(4:6,4:6), ~] = correctionPitot_airspeed(x_lin, P_lin(4:6,4:6), pitot_data(index_pitot, 2:end), nas_struct.sigma_pitot2, settings.OMEGA);
            input_data{ii, 9} = pitot_data(index_pitot, 2);
        end

        output_data{ii, 1} = 0;
        output_data{ii, 2} = [x_lin x_quat];
        output_data{ii, 3} = blkdiag(P_lin, P_quat);

        state_in(1:6) = x_lin;
        state_in(7:13) = x_quat;
        P_in(1:6, 1:6) = P_lin;
        P_in(7:12, 7:12) = P_quat;
    end

    % NAS INPUT
    disp("NAS INPUT");
    writelines("NASState nasInputs[] = {",Input_StateData_File, 'WriteMode', 'append');
    for ii = 2:length(t_nas)
        writelines("{" + ii + ",{", Input_StateData_File, 'WriteMode', 'append');
        writecell(input_data(ii, 2), Input_StateData_File, 'WriteMode', 'append');
        writelines("}},", Input_StateData_File, 'WriteMode', 'append');
    end
    writelines("};", Input_StateData_File, 'WriteMode', 'append');

    % ACCELEROMETER
    disp("ACCELEROMETER");
    disp(input_data(1:5, [1 4]));
    if isempty(algorithms) || any(ismember(algorithms, {'predict_acc', 'complete'}))
        writelines("AccelerometerData acc[] = {", AccData_File, 'WriteMode', 'append');
        for ii = 2:length(t_nas)
            writelines("{", AccData_File, 'WriteMode', 'append');
            writecell(input_data(ii, [1 4]), AccData_File, 'WriteMode', 'append');
            writelines("},", AccData_File, 'WriteMode', 'append');
        end
        writelines("};", AccData_File, 'WriteMode', 'append');
    end
    
    % GYROSCOPE
    disp("GYROSCOPE");
    if isempty(algorithms) || any(ismember(algorithms, {'predict_gyro', 'complete'}))
        writelines("GyroscopeData gyro[] = {", GyroData_File, 'WriteMode', 'overwrite');
        for ii = 2:length(t_nas)
            writelines("{", GyroData_File, 'WriteMode', 'append');
            writecell(input_data(ii, [1 5]), GyroData_File, 'WriteMode', 'append');
            writelines("},", GyroData_File, 'WriteMode', 'append');
        end
        writelines("};", GyroData_File, 'WriteMode', 'append');
    end

    % GPS
    disp("GPS");
    if isempty(algorithms) || any(ismember(algorithms, {'correct_gps', 'complete'}))
        writelines("GPSData gps[] = {", GPSData_File, 'WriteMode', 'overwrite');
        for ii = 2:length(t_nas)
            writelines("{", GPSData_File, 'WriteMode', 'append');
            writecell(input_data(ii, [1 6]), GPSData_File, 'WriteMode', 'append');
            writelines("},", GPSData_File, 'WriteMode', 'append');
        end
        writelines("};", GPSData_File, 'WriteMode', 'append');
    end
    
    % BAROMETER
    disp("BAROMETER");
    if isempty(algorithms) || any(ismember(algorithms, {'correct_baro', 'complete'}))
        writelines("PressureData baro[] = {", BaroData_File, 'WriteMode', 'overwrite');
        for ii = 2:length(t_nas)
            writelines("{", BaroData_File, 'WriteMode', 'append');
            writecell(input_data(ii, [1 7]), BaroData_File, 'WriteMode', 'append');
            writelines("},", BaroData_File, 'WriteMode', 'append');
        end
        writelines("};", BaroData_File, 'WriteMode', 'append');
    end
    disp(input_data(10, [1 7]));

    % MAGNETOMETER
    disp("MAGNETOMETER");
    if isempty(algorithms) || any(ismember(algorithms, {'correct_mag', 'complete'}))
        writelines("MagnetometerData mag[]  = {", MagData_File, 'WriteMode', 'overwrite');
        for ii = 2:length(t_nas)
            writelines("{", MagData_File, 'WriteMode', 'append');
            writecell(input_data(ii, [1 8]), MagData_File, 'WriteMode', 'append');
            writelines("},", MagData_File, 'WriteMode', 'append');
        end
        writelines("};", MagData_File, 'WriteMode', 'append');
    end
    
    % PITOT
    disp("PITOT");
    if isempty(algorithms) || any(ismember(algorithms, {'correct_pitot', 'complete'}))
        writelines("PitotData pitot[] = {", PitotData_File, 'WriteMode', 'overwrite');
        for ii = 2:length(t_nas)
            writelines("{", PitotData_File, 'WriteMode', 'append');
            writecell(input_data(ii, [1 9]), PitotData_File, 'WriteMode', 'append');
            writelines("},", PitotData_File, 'WriteMode', 'append');
        end
        writelines("};", PitotData_File, 'WriteMode', 'append');
    end
    
    % NAS OUTPUT
    disp("NAS OUTPUT");
    writelines("NASState nasOutputs[] = {",Output_StateData_File, 'WriteMode', 'append');
    for ii = 2:length(t_nas)
        writelines("{" + ii + ",{", Output_StateData_File, 'WriteMode', 'append');
        writecell(output_data(end, 2), Output_StateData_File, 'WriteMode', 'append');
        writelines("}},", Output_StateData_File, 'WriteMode', 'append');
    end
    writelines("};", Output_StateData_File, 'WriteMode', 'append');
end