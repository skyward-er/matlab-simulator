
function exportNAS(nas_data, settings, target)
    if strcmpi(settings.scenario, 'descent')
        error("Catch test data cannot be used in descent case!");
    end
    
    root_folder_name    = "exports";
    nas_folder_name     = "nas";
    mission_folder_name = "mission_" + settings.mission;
    
    out_path = ".." + ...
                "/" + root_folder_name + ...
                "/" + nas_folder_name + ...
                "/" + mission_folder_name + ...
                "/" + target;

    disp("Exporting data to: " + out_path);

    if ~exist(out_path, "dir")
        mkdir(out_path)
    end

    %% Configuration file

    std_magneticField; % Regenerate magnetic field data
    nas_struct = settings.nas;

    NAS_config_table = table;
    NAS_config_values = [
        1/settings.frequencies.NASFrequency, ... NAS_T
        settings.nas.sigma_beta,             ... SIGMA_BETA
        settings.nas.sigma_w,                ... SIGMA_W
        settings.nas.sigma_mag,              ... SIGMA_MAG
        settings.nas.sigma_GPS,              ... SIGMA_GPS
        settings.nas.sigma_baro,             ... SIGMA_BAR
        settings.nas.QLinear(1,1),           ... SIGMA_POS
        settings.nas.QLinear(end, end),      ... SIGMA_VEL
        settings.nas.sigma_pitot2,           ... SIGMA_PITOT
        0.01, 0.01,                          ... P_POS, P_POS_VERTICAL
        0.01, 0.01,                          ... P_VEL, P_VEL_VERTICAL
        0.01, 0.01,                          ... P_ATT, P_BIAS
        single(XYZ0*0.01)',                  ... MAG_NED
        nas_struct.stopPitotAltitude         ... PITOT_STOP_ALTITUDE
    ];

    NAS_config_keys = {
        'NAS_T',                ...
        'SIGMA_BETA',           ...
        'SIGMA_W',              ...
        'SIGMA_MAG',            ...
        'SIGMA_GPS',            ...
        'SIGMA_BAR',            ...
        'SIGMA_POS',            ...
        'SIGMA_VEL',            ...
        'SIGMA_PITOT',          ...
        'P_POS',                ...
        'P_POS_VERTICAL',       ...
        'P_VEL',                ...
        'P_VEL_VERTICAL',       ...
        'P_ATT',                ...
        'P_BIAS',               ...
        'MAG_NED_X',            ...
        'MAG_NED_Y',            ...
        'MAG_NED_Z',            ...
        'PITOT_STOP_ALTITUDE'   ...
    };

    for i = 1:size(NAS_config_values,2)
        NAS_config_table(1,i) = table(NAS_config_values(1,i));
    end
    
    NAS_config_table.Properties.VariableNames = NAS_config_keys;
    config_file = out_path + "/config";

    %% Generate input files

    input_state_file  = out_path + "/input";
    output_state_file = out_path + "/output";
    acc_file          = out_path + "/acc";
    gyro_file         = out_path + "/gyro";
    gps_file          = out_path + "/gps";
    baro_file         = out_path + "/baro";
    mag_file          = out_path + "/mag";
    pitot_file        = out_path + "/pitot";
    steps_file        = out_path + "/steps";

    %% Data types

    cpp_data_types = dictionary(       ...
        'input' , 'NASState',          ...
        'output', 'NASState',          ...
        'acc'   , 'AccelerometerData', ...
        'gyro'  , 'GyroscopeData',     ...
        'gps'   , 'GPSData',           ...
        'baro'  , 'PressureData',      ...
        'mag'   , 'MagnetometerData',  ...
        'pitot' , 'PitotData'          ...
    );

    data_types = dictionary(  ...
        'cpp', cpp_data_types ...
    );

    %% Export everything

    export_config(NAS_config_table, config_file, target);

    export_state(nas_data.input,  input_state_file,  target, 'input' , data_types);
    export_state(nas_data.output, output_state_file, target, 'output', data_types);

    export_sensor_data(nas_data.acc,   acc_file,   target, 'acc'  , data_types);
    export_sensor_data(nas_data.gyro,  gyro_file,  target, 'gyro' , data_types);
    export_sensor_data(nas_data.gps,   gps_file,   target, 'gps'  , data_types);
    export_sensor_data(nas_data.baro,  baro_file,  target, 'baro' , data_types);
    export_sensor_data(nas_data.mag,   mag_file,   target, 'mag'  , data_types);
    export_sensor_data(nas_data.pitot, pitot_file, target, 'pitot', data_types);

    export_steps(nas_data.steps, steps_file, target);
end

function export_config(data, file, target)
    disp("Exporting config to: " + file + "...");
    switch target
        case 'csv'
            file = file + ".csv";
            clear_file(file);
            writetable(data, file);
        case 'cpp'
            file = file + ".h";
            clear_file(file);
            write_cpp_header(file);
            write("NASConfig nasConfig {",                                           file);
            write("    "   + data.NAS_T            + ", ///< T",                     file);
            write("    "   + data.SIGMA_BETA       + ", ///< SIGMA_BETA",            file);
            write("    "   + data.SIGMA_W          + ", ///< SIGMA_W",               file);
            write("    "   + 10                    + ", ///< SIGMA_ACC [simulated]", file);
            write("    "   + data.SIGMA_MAG        + ", ///< SIGMA_MAG",             file);
            write("    "   + data.SIGMA_GPS        + ", ///< SIGMA_GPS",             file);
            write("    "   + data.SIGMA_BAR        + ", ///< SIGMA_BAR",             file);
            write("    "   + data.SIGMA_POS        + ", ///< SIGMA_POS",             file);
            write("    "   + data.SIGMA_VEL        + ", ///< SIGMA_VEL",             file);
            write("    "   + data.SIGMA_PITOT      + ", ///< SIGMA_PITOT",           file);
            write("    "   + data.P_POS            + ", ///< P_POS",                 file);
            write("    "   + data.P_POS_VERTICAL   + ", ///< P_POS_VERTICAL",        file);
            write("    "   + data.P_VEL            + ", ///< P_VEL",                 file);
            write("    "   + data.P_VEL_VERTICAL   + ", ///< P_VEL_VERTICAL",        file);
            write("    "   + data.P_ATT            + ", ///< P_ATT",                 file);
            write("    "   + data.P_BIAS           + ", ///< P_BIAS",                file);
            write("    "   + 6                     + ", ///< SATS_NUM [simulated]",  file);
            write("    { " + data.MAG_NED_X + ", " + ...
                             data.MAG_NED_Y + ", " + ...
                             data.MAG_NED_Z + " }" + ", ///< NED_MAG",               file);
            write("};",                                                              file);
    end
end

function export_state(data, file, target, type, data_types)
    disp("Exporting " + type + " to: " + file + "...");
    switch target
        case 'csv'
            file = file + ".csv";
            clear_file(file);
            writetable(cell2table(data), file);
        case 'cpp'
            file = file + ".h";
            clear_file(file);
            write_cpp_header(file);
            cpp_data_types = data_types('cpp');
            data_type      = cpp_data_types(type);
            write(data_type + " " + type + "[] = {", file);
            for idx = 2:length(data)
                timestamp = data(idx, 1);
                state_input = cell2mat(data(idx, 2));
                state_input_strs = arrayfun(@(x) num2str(x), state_input, 'UniformOutput', false);
                concat_state_input = strjoin(state_input_strs, ", ");
                write("    {",                                   file);
                write("        " + timestamp + ",",              file);
                write("        {",                               file);
                write("            " + concat_state_input + ",", file);
                write("        }",                               file);
                write("    },",                                  file);

                if mod(idx, 100) == 0
                    disp("Exporting " + type + " to: " + file + " - " + idx + "/" + length(data));
                end
            end
            write("};", file);
    end
    disp("Done exporting " + type + " to: " + file);
end

function export_sensor_data(data, file, target, type, data_types)
    disp("Exporting " + type + " to: " + file + "...");
    switch target
        case 'csv'
            file = file + ".csv";
            clear_file(file);
            writematrix(data, file);
        case 'cpp'
            file = file + ".h";
            clear_file(file);
            write_cpp_header(file);
            cpp_data_types = data_types('cpp');
            data_type      = cpp_data_types(type);
            write(data_type + " " + type + "[] = {", file);
            for idx = 2:length(data)
                sensor_input = data(idx, :);
                sensor_input_strs = arrayfun(@(x) num2str(x), sensor_input, 'UniformOutput', false);
                concat_sensor_input = strjoin(sensor_input_strs, ", ");

                write("    {",                   file);
                write(concat_sensor_input + ",", file);
                write("    },",                  file);

                if mod(idx, 100) == 0
                    disp("Exporting " + type + " to: " + file + " - " + idx + "/" + length(data));
                end
            end
            write("};", file);
    end
    disp("Done exporting " + type + " to: " + file);
end

function export_steps(data, file, target)
    disp("Exporting steps to: " + file + "...");
    switch target
        case 'csv'
            file = file + ".csv";
            clear_file(file);
            writetable(cell2table(data), file);
        case 'cpp'
            file = file + ".h";
            clear_file(file);
            write_cpp_header(file);
            write("struct NASPredictionSteps {", file);
            % After acceleration prediction
            write("float acc_x;",        file);
            write("float acc_y;",        file);
            write("float acc_z;",        file);
            write("float acc_vx;",       file);
            write("float acc_vy;",       file);
            write("float acc_vz;",       file);
            % After gyro prediction
            write("float gyro_gx;",      file);
            write("float gyro_gy;",      file);
            write("float gyro_gz;",      file);
            write("float gyro_gw;",      file);
            write("float gyro_gbx;",     file);
            write("float gyro_gby;",     file);
            write("float gyro_gbz;",     file);
            % After gps correction
            write("float gps_x;",        file);
            write("float gps_y;",        file);
            write("float gps_z;",        file);
            write("float gps_vx;",       file);
            write("float gps_vy;",       file);
            write("float gps_vz;",       file);
            % After barometer correction
            write("float baro_x;",       file);
            write("float baro_y;",       file);
            write("float baro_z;",       file);
            write("float baro_vx;",      file);
            write("float baro_vy;",      file);
            write("float baro_vz;",      file);
            % After magnetometer correction
            write("float mag_gx;",       file);
            write("float mag_gy;",       file);
            write("float mag_gz;",       file);
            write("float mag_gw;",       file);
            write("float mag_gbx;",      file);
            write("float mag_gby;",      file);
            write("float mag_gbz;",      file);
            % After pitot correction
            write("float pitot_x;",      file);
            write("float pitot_y;",      file);
            write("float pitot_z;",      file);
            write("float pitot_vx;",     file);
            write("float pitot_vy;",     file);
            write("float pitot_vz;",     file);
            write("};",                  file)

            write("NASPredictionSteps steps[] {", file);
            for idx = 2:length(data)
                steps     = data(idx);
                acc_lin   = arrayfun(@(x) num2str(x), steps.acc_lin,   'UniformOutput', false);
                gyro_quat = arrayfun(@(x) num2str(x), steps.gyro_quat, 'UniformOutput', false);
                gps_lin   = arrayfun(@(x) num2str(x), steps.gps_lin,   'UniformOutput', false);
                baro_lin  = arrayfun(@(x) num2str(x), steps.baro_lin,  'UniformOutput', false);
                mag_quat  = arrayfun(@(x) num2str(x), steps.mag_quat,  'UniformOutput', false);
                pitot_lin = arrayfun(@(x) num2str(x), steps.pitot_lin, 'UniformOutput', false);

                strs = strjoin([acc_lin gyro_quat gps_lin baro_lin mag_quat pitot_lin], ", ");

                write("    {",    file);
                write(strs + ",", file);
                write("    },",   file);

                
                if mod(idx, 100) == 0
                    disp("Exporting steps to: " + file + " - " + idx + "/" + length(data));
                end
            end
            write("};", file);
        end
    disp("Done exporting steps to: " + file + " - " + idx + "/" + length(data));
end

function write(text, file)
    writelines(text, file, 'WriteMode', 'append');
end

function clear_file(file)
    if isfile(file)
        delete(file);
    end
end

function write_cpp_header(file)
    write('/* Copyright (c) 2024 Skyward Experimental Rocketry                             ', file);
    write(' * Author: Davide Basso                                                         ', file);
    write(' *                                                                              ', file);
    write(' * Permission is hereby granted, free of charge, to any person obtaining a copy ', file);
    write(' * of this software and associated documentation files (the "Software"), to deal', file);
    write(' * in the Software without restriction, including without limitation the rights ', file);
    write(' * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell    ', file);
    write(' * copies of the Software, and to permit persons to whom the Software is        ', file);
    write(' * furnished to do so, subject to the following conditions:                     ', file);
    write(' *                                                                              ', file);
    write(' * The above copyright notice and this permission notice shall be included in   ', file);
    write(' * all copies or substantial portions of the Software.                          ', file);
    write(' *                                                                              ', file);
    write(' * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR   ', file);
    write(' * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,     ', file);
    write(' * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE" ', file);
    write(' * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER       ', file);
    write(' * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,', file);
    write(' * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN    ', file);
    write(' * THE SOFTWARE.                                                                ', file);
    write(' */                                                                             ', file);
    write('                                                                                ', file);
    write('/* This file was automatically generated, do not edit manually! */              ', file);
    write('                                                                                ', file);
    write('#include <algorithms/NAS/NASConfig.h>                                           ', file);
    write('#include <algorithms/NAS/NASState.h>                                            ', file);
    write('#include <sensors/SensorData.h>                                                 ', file);
    write('#include <sensors/analog/Pitot/PitotData.h>                                     ', file);
    write('                                                                                ', file);
    write('                                                                                ', file);
    write('using namespace Boardcore;                                                      ', file);
    write('                                                                                ', file);
end