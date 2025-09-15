%% export_HILdataPRF
%
% HELP:
%
% script that exports data for PRF hardware in the loop and software
% testing for parafoil system
%
% check also export_HILdataABK for air brakes HIL and software testing

folder = "ConfigData/PRFConfig";
if ~exist(ConDataPath+"/"+folder,"dir")
    mkdir(ConDataPath+"/"+folder)
end

% Configuration file
configPRF_export_table = table;
% [target_LAT, target_LON] = ned2geodetic(settings.payload.target(1),...
%     settings.payload.target(2),environment.z0,environment.lat0,...
%     environment.lon0,environment.z0,wgs84Ellipsoid);
target_LAT = 41.80790524057098; 
target_LON = 14.057047761535994;
configValues = [target_LAT, target_LON, contSettings.payload.mult_EMC,...
    contSettings.payload.d, rocket.parachutes(2,2).controlParams.Kp,...
    rocket.parachutes(2,2).controlParams.Ki];
configPRFvarNames = {'TARGET_LAT','TARGET_LON','MULT_EMC','D_EMTP','Kp', 'Ki'};
for i = 1:size(configValues,2)
    configPRF_export_table(1,i) = table(configValues(1,i));
end
configPRF_export_table.Properties.VariableNames = configPRFvarNames;
writetable(configPRF_export_table,ConDataPath+"/"+folder+"/PRF_configs_data.csv")