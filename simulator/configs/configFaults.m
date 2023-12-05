%{

sensor fault configuration script


%}

% how many faults do you want to simulate?
settings.fault_sim.N_faulty_sensors = -1; % if set to -1 it will go to manual fault setting, otherwise it will generate random faults at a set of random sensors


if settings.fault_sim.N_faulty_sensors == -1
    settings.fault_sim.selected_sensors = [2 3];
    settings.fault_sim.fault_type = ["no fault", "no fault", "no fault"];
end

% fault parameters
settings.fault_sim.max_offset = 1300;       %Pa
settings.fault_sim.min_offset = 200;        %Pa
settings.fault_sim.max_degradation = 1300;  %Pa
settings.fault_sim.min_degradation = 200;   %Pa
settings.fault_sim.min_drift_value = 20;    %Pa
settings.fault_sim.max_drift_value = 200;   %Pa



selected_sensors = [];
fault_type = ["no fault", "no fault", "no fault"];
if settings.fault_sim.N_faulty_sensors == -1 % the sensor fault is setted up manually in configfaults
    selected_sensors = settings.fault_sim.selected_sensors;
    fault_type = settings.fault_sim.fault_type;
else % fault generation is done randomly between a set of parameters in configfaults
    for i = 1:settings.fault_sim.N_faulty_sensors
        rand_fault = randi(4); 
       
        continue_generate = true;
        while continue_generate
            continue_generate = false;
            rand_sensor = randi(3);
            if length(selected_sensors) > 0
                iterations_for_selection = length(selected_sensors);
            else
                iterations_for_selection = 1;
            end
            for j = 1:iterations_for_selection
                if isempty(selected_sensors) || ~isequal(rand_sensor, selected_sensors(j))
                    selected_sensors = [selected_sensors, rand_sensor];
                else
                    continue_generate = true;
                end
            end
        end
        switch rand_fault
            case 1
                settings.fault_sim.fault_type(rand_sensor) = "offset";
            case 2
                settings.fault_sim.fault_type(rand_sensor) = "degradation";
            case 3
                settings.fault_sim.fault_type(rand_sensor) = "freezing";
            case 4
                settings.fault_sim.fault_type(rand_sensor) = "drift";
        end
    end
end
