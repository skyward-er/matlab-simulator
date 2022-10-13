%{

extract only useful parameters from MSA toolkit:

%}

% set which fields you want to remove from where
fieldsToRemove_settings = {'x_final','y_final', ...
                  'boatL', 'boatD', ...
                  'Vx_final','Vy_final', ...
                  'Local', ...
                  'lpin', 'lrampa', ...
                  'pitotConeLength','pitotDiameter','pitotLength', ...
                  'mNoMot', ...
                  'pMod','cMod', ...
                  'limLat','limLon',...
                  };

switch contSettings.algorithm
    case "interp"
        fieldsToRemove_contSettings = {'data_trajectories'};
    case "PID_2021"

end

rmfield(settings, fieldsToRemove_settings);
rmfield(contSettings, fieldsToRemove_contSettings);

%% clear useless variables
clear GeometryE GeometryF...
    fun...
    fieldsToRemove_contSettings fieldsToRemove_settings...
    iMotors...
    reference trajectories_saving V_rescale...
    i ii ans conf...