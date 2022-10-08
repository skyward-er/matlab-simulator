%{

folder path configuration script

%}

% Retrieve MSA-Toolkit rocket data
dataPath = strcat('../data/msa-toolkit/data/', settings.mission);
addpath(dataPath);
simulationsData; 

commonFunctionsPath = '../data/msa-toolkit/commonFunctions';
addpath(genpath(commonFunctionsPath))

% Retrieve Control rocket data
ConDataPath = strcat('../data/', settings.mission);
addpath(ConDataPath);
run(strcat('config', settings.mission));

% Control common functions
commonFunctionsPath = '../commonFunctions';
addpath(genpath(commonFunctionsPath))