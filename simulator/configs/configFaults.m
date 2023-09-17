%{

sensor fault configuration script


%}

% how many faults do you want to simulate?
N_faulty_sensors = 2;

% select which type of fault: "offset" "degradation" "freezing"
degradation_type = "offset";


% select which sensors will be faulty
if N_faulty_sensors == 1
settings.which_sens = ceil(rand(1)*3);
elseif N_faulty_sensors ==2
    settings.which_sens = ceil(rand(2,1)*3);
    while settings.which_sens(1)==settings.which_sens(2)
        settings.which_sens = ceil(rand(1,2)*3);
    end
else 
    settings.which_sens = [1,2,3];
end

% fault parameters
max_offset = 1300; %Pa
min_offset = 200; %Pa
max_degradation = 1300; %Pa
min_degradation = 200; %Pa


setting.offset_value_1 = round((max_offset-min_offset)*rand() + min_offset);
setting.offset_value_2 = round((max_offset-min_offset)*rand() + min_offset);
setting.offset_value_3 = round((max_offset-min_offset)*rand() + min_offset);


setting.degradation_value_1 = round((max_offset-min_offset)*rand() + min_offset);
setting.degradation_value_2 = round((max_degradation-min_degradation)*rand() + min_degradation);
setting.degradation_value_3 = round((max_degradation-min_degradation)*rand() + min_degradation);

% for i = 1:3
%         rand_fault = randi(3);
%     switch rand_fault
%         case 1
%             setting.fault_type(i) = "offset";
%         case 2
%             setting.fault_type(i) = "degradation";
%         case 3
%             setting.fault_type(i) = "freezing";
%     end
% end
% switch fault_type
% 
%     case "offset"
% 
%     case "degradation"
% 
%     case "freezing"
% 
% end


%% SENSOR FAULT DETECTION PARAMETERS
settings.SVM_1.Scale = 3.252930891288769;
settings.SVM_1.Mu = [1306.72004416967	0.351190954475099	1.99867039117899	-0.0185223927903996	7.45589640537251	12.6303641023070];
settings.SVM_1.Sigma = [1055.13277492127	0.0457124377157068	1.83336893903227	0.0874316265177542	1.19779014092547	0.999724384795152];
settings.SVM_1.Beta = [0.0538048411163895 1.21829189675103 -0.0135044925652423 -2.46984492432602 -1.74699069029220 0.236220477009252]';
settings.SVM_1.Bias = -0.945291058126819;


settings.SVM_2.Scale = 0.775015678995177;
settings.SVM_2.Mu = [1.69681676472682	261.159444699530	0.0114112487465149	-0.0168260044537472	2.13558588178350];
settings.SVM_2.Sigma = [0.208238288492930	356.048735537809	0.156943288015148	0.480743200014727	0.912588579044107];
settings.SVM_2.Beta = [-1.45789357962751 0.149212230274532 0.543310785123195 0.366578333245410 -0.0334769996901740]';
settings.SVM_2.Bias = -4.556178429828644;


settings.SVM_1.N_sample =  50;% window size of features that are not fft related
settings.SVM_1.N_sample_fft = 10;% window size of features that are not fft related

settings.SVM_2.N_sample = 50;
settings.SVM_1.takeoff_shadowmode = 6; % seconds to wait before starting detection

settings.SVM_2.takeoff_shadowmode = -1;

settings.faulty_sensors = [false, false, false]; % if true, then it's faulty, if false, all good