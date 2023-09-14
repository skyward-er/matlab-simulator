%{
    export get drag coefficients
%}


switch settings.mission
    case  {"Gemini_Portugal_October_2023", "Gemini_Roccaraso_September_2023"}
CD_coefficients  = [contSettings.coeff_Cd.n000,contSettings.coeff_Cd.n100,...
    contSettings.coeff_Cd.n200,contSettings.coeff_Cd.n300,contSettings.coeff_Cd.n400,...
    contSettings.coeff_Cd.n500,contSettings.coeff_Cd.n600];
CD_coeff_table = table;
varNames = {'n000','n100','n200','n300','n400','n500','n600'};
for i = 1:size(CD_coefficients,2)
    CD_coeff_table(:,i) = table(CD_coefficients(:,i));
end
CD_coeff_table.Properties.VariableNames = varNames;
writetable(CD_coeff_table,ConDataPath+"/HIL_CPP_files_ABK/MTR_CDcoeffShutDown_"+settings.mission+".csv")

% passare anche la massa iniziale (valori di inizializzazione) - discutere
% anche delle shadowmode - 
% aggiungere che deve beccare 5 volte consecutive l'apogeo predetto sopra
% al target
% second file: configuration for the air brakes
configMTR_export_table = table;
configValues = [];
configMTRvarNames = {'SHUTDOWN_LOWER_SHADOWMODE','SHUTDOWN_HIGHER_SHADOWMODE'};
for i = 1:size(configValues,2)
    configMTR_export_table(1,i) = table(configValues(1,i));
end
configMTR_export_table.Properties.VariableNames = configMTRvarNames;
writetable(configMTR_export_table,ConDataPath+"/HIL_CPP_files_ABK/ABK_configABK_"+settings.mission+".csv")


end