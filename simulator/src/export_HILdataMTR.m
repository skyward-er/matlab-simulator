%{
    export get drag coefficients
%}
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