%% export_HILdataABK
%
% HELP:
%
% script that exports data for ABK hardware in the loop and software
% testing for air brakes and motor shutdown
%
% check also export_HILdataPRF for parafoil HIL and software testing

folder = "ConfigData/ABKConfig";
if ~exist(ConDataPath+"/"+folder,"dir")
    mkdir(ConDataPath+"/"+folder)
end
% first file: trajectories, set in the following way:
% first column - heights;
% next N_mass columns - vertical velocity with closed air brakes;
% next N_mass columns - vertical velocity with open air brakes;
reference_export = zeros(size(contSettings.reference.Z,1),1+2*size(contSettings.reference.Vz,2));
reference_export(:,1) = contSettings.reference.Z;
varNames{1,1} = 'Heights';
for i = 1:size(contSettings.reference.Vz,1)
    for j = 1:size(contSettings.reference.Vz,2)
        reference_export(:,1+(i-1)*size(contSettings.reference.Vz,2)+j) = contSettings.reference.Vz{i,j};
        if i == 1
            varNames{1,1+(i-1)*size(contSettings.reference.Vz,2)+j} = ['Vz_closed_m',num2str(contSettings.masses_vec(j))];
        else
            varNames{1,1+(i-1)*size(contSettings.reference.Vz,2)+j} = ['Vz_open_m',num2str(contSettings.masses_vec(j))];
        end
    end
end
varNames = replace(varNames,'.','_');
for i = 1:size(reference_export,2)
    reference_export_table(:,i) = table(reference_export(:,i));
end
reference_export_table.Properties.VariableNames = varNames;
writetable(reference_export_table,ConDataPath+"/"+folder+"/ABK_reference_trajctories.csv")

% second file: configuration for the air brakes
configABK_export_table = table;
configValues = [settings.frequencies.arbFrequency, rocket.airbrakes.maxMach, contSettings.ABK_shutdown_delay, contSettings.filterMinAltitude, contSettings.filterMaxAltitude, ...
    contSettings.filter_coeff0, contSettings.criticalAltitude, contSettings.reference.deltaZ, contSettings.masses_vec(1), contSettings.dmass, contSettings.N_forward, ...
    contSettings.ABK.PID_ref, contSettings.ABK.PID_coeffs(1), contSettings.ABK.PID_coeffs(2), contSettings.ABK.PID_coeffs(3), ...
    simOutput.sensors.mea.mass(end)];
configABKvarNames = {'ABK_FREQUENCY','MACH_LIMIT', 'SHADOW_MODE_TIMEOUT ', 'FILTER_MINIMUM_ALTITUDE ', 'FILTER_MAXIMUM_ALTITUDE ', ...
    'STARTING_FILTER_VALUE', 'ABK_CRITICAL_ALTITUDE', 'DZ', 'INITIAL_MASS', 'DM', 'N_FORWARD', ...
    'PID_REF', 'KP', 'KI', 'KD', ...
    'ESTIMATED_MASS'};
for i = 1:size(configValues,2)
    configABK_export_table(1,i) = table(configValues(1,i));
end
configABK_export_table.Properties.VariableNames = configABKvarNames;
writetable(configABK_export_table,ConDataPath+"/"+folder+"/ABK_configs_data.csv")

% third file: input extrapolation:
% first column - timestamps of the NAS system
% second column - data with vertical position of the NAS;
% third column - data with vertical velocity of the NAS;
startIdx = sum(simOutput.sensors.nas.time <= 1.5*simOutput.state_lastTimes(1));
endIdx = sum(simOutput.sensors.nas.time <= simOutput.state_lastTimes(3));
ABKInput_export_table = table;
ABKInput = zeros(size(simOutput.sensors.nas.time(startIdx:endIdx)'));
ABKInput(:,1) = -simOutput.sensors.nas.states(startIdx:endIdx,3);
ABKInput(:,2) = -simOutput.sensors.nas.states(startIdx:endIdx,6);
ABKInput_varNames = {'Z','Vz'};
for i = 1:size(ABKInput,2)
    ABKInput_export_table(:,i) = table(ABKInput(:,i));
end
ABKInput_export_table.Properties.VariableNames = ABKInput_varNames;
writetable(ABKInput_export_table,ConDataPath+"/"+folder+"/ABK_input.csv")

% fourth file: input extrapolation:
% first column - data with air brakes timestamps;
% second column - data with air brakes output of the simulation;
%
% to simplify the testing the ABK algorithm is re-run with the data
% of the third file (input)
% recall the initial conditions for the controls
configControl;
configControlParams;
configReferences;
contSettings.ABK.saturation = false;
contSettings_choosemass = trajectoryChoice_mass(simOutput.sensors.mea.mass(end),contSettings);
ABK_recall = zeros(length(ABKInput(:,2)),1);
int_error = zeros(length(ABKInput(:,2)),1);
int_error_old = 0;
ABK_recall_old = 0;
for i = 1:length(ABKInput(:,2))
    [ABK_recall(i),int_error(i),contSettings_choosemass] = control_Interp_PID(ABKInput(i,1),ABKInput(i,2),settings,contSettings_choosemass,ABK_recall_old,int_error_old);
    ABK_recall_old = ABK_recall(i);
    int_error_old = int_error(i);
end
ABK_recall = ABK_recall/settings.servo.maxAngle;
outputABK_export_table = table;
ABK_output = [ABK_recall int_error];
ABK_varNames = {'Percentage', 'int_error'};
for i = 1:size(ABK_output,2)
    outputABK_export_table(:,i) = table(ABK_output(:,i));
end
outputABK_export_table.Properties.VariableNames = ABK_varNames;
writetable(outputABK_export_table,ConDataPath+"/"+folder+"/ABK_output.csv")
