function [sensorData, sensorTot, ada_settings, flagMajorityApogee, flagMajorityOpenPara] = run_Majority_ADA(Tf, ADA_N_instances, ada_settings, ada_frequency, sensorData, sensorTot, currentState, engineT0)

    % Time vector for the current algorithm timestamps to simulate
    t_ada = sensorTot.ada.time(end):1/ada_frequency:Tf(end);

    % Set the matrices that will contain the states of the algorithms
    % during the updates.
    % The first value of each matrix will be the value at the last
    % timestamp of the previous iteration of the algorithm
    xp = zeros(length(t_ada), 3, ADA_N_instances);
    P = zeros(3, 3, length(t_ada), ADA_N_instances);
    xv = zeros(length(t_ada), 2, ADA_N_instances);
    for jj = 1:ADA_N_instances
        xp(1,:,jj)   = sensorData.ada{jj}.xp(end,:);
         P(:,:,1,jj) = sensorData.ada{jj}.P(:,:,end);
        xv(1,:,jj)   = sensorData.ada{jj}.xv(end,:);
    end

    % The majority flags are set as the previous iteration value
    flagMajorityApogee = ada_settings.flag_apo;
    flagMajorityOpenPara = ada_settings.flagOpenPara;

    % Initialize flag matrices to hold the values of the flag for the
    % current iterations
    flagApogeeMat = false(length(t_ada), ADA_N_instances);
    flagApogeeMat(1,:) = sensorTot.ada.flagApogee(end,:);
    flagOpenParaMat = false(length(t_ada), ADA_N_instances);
    flagOpenParaMat(1,:) = sensorTot.ada.flagOpenPara(end,:);

    % If the length is 1 it means that t_ada only contains
    % sensorTot.ada.time(end), which has already been simulated during the
    % previous simulator iteration.
    % If the length is <= 0 then there is a problem with the way t_ada was
    % computed.
    if length(t_ada) > 1

        % Initialize dt
        dt = diff(t_ada(1:2));

        for ii = 2:length(t_ada)

            % Update all the instances of the algorithm
            for jj = 1:ADA_N_instances
                [xp(ii,:,jj), P(:,:,ii,jj), xv(ii,:,jj), sensorData.ada{jj}.lastBaroTimestamp] = ...
                 run_ADA_update(t_ada(ii), dt, ada_settings, xp(ii-1, :, jj), P(:,:,ii-1,jj), ...
                 sensorData.barometer_sens{jj}, sensorData.ada{jj}.lastBaroTimestamp, @getAltitude, @getvelocity);
            end

            % If the rocket is flying, perform the checks for all instances for apogee
            if currentState ~= 1 % state 1 is the on_ground state
                for jj = 1:ADA_N_instances
                    if ada_settings.flag_apo == false && flagApogeeMat(ii-1, jj) == false
                        if xv(ii, 2, jj) < ada_settings.v_thr
                            sensorData.ada{jj}.counter = sensorData.ada{jj}.counter + 1;
                        else
                            sensorData.ada{jj}.counter = 0;
                        end

                        if sensorData.ada{jj}.counter >= ada_settings.count_thr
                            % If the apogee is detected while still in
                            % shadowmode issue a warning to notify of it
                            % happening
                            if t_ada(ii) < (ada_settings.shadowmode + engineT0)
                                % warning("ADA %d detected an apogee while still in shadowmode", jj);
                            else
                                flagApogeeMat(ii, jj) = true;
                            end
                        end
                    elseif flagApogeeMat(ii-1, jj) == true
                        flagApogeeMat(ii, jj) = true;
                    end
                end
            end

            % Perform the checks for all instances for parachute opening
            % only if the apogee has already been detected
            for jj = 1:ADA_N_instances
                if ada_settings.flag_apo
                    if ada_settings.flagOpenPara == false
                        if xv(ii,1,jj) < ada_settings.z_cut
                            sensorData.ada{jj}.paraCounter = sensorData.ada{jj}.paraCounter + 1;
                        else
                            sensorData.ada{jj}.paraCounter = 0;
                        end
                    end
                end
                if sensorData.ada{jj}.paraCounter >= ada_settings.altitude_confidence_thr
                    flagOpenParaMat(ii,jj) = true;
                end
            end

            % If more than 50% of the instances detect apogee then the apogee flag
            % is set to true
            if ada_settings.flag_apo == false && sum(flagApogeeMat(ii,:)) >= ceil(ADA_N_instances/2)
                flagMajorityApogee = true;
                ada_settings.t_ada = t_ada(ii);
            end

            % If more than 50% of the instances detect parachute opening altitude
            % then the OpenPara flag is set to true
            if ada_settings.flag_apo && sum(flagOpenParaMat(ii,:)) >= ceil(ADA_N_instances/2)
                flagMajorityOpenPara = true;
                ada_settings.t_para = t_ada(ii);
            end

        end
    end

    ada_settings.flag_apo = flagMajorityApogee;
    ada_settings.flagOpenPara = flagMajorityOpenPara;

    % Update all the values in sensorData, so they can be used during the
    % next iteration (the choice of jj instead of ii is for consistency with previous code)
    for jj = 1:ADA_N_instances
        sensorData.ada{jj}.xp = xp(:,:,jj);
        sensorData.ada{jj}.P = P(:,:,:,jj);
        sensorData.ada{jj}.xv = xv(:,:,jj);
    end
    
    % Update the sensorTot struct, used for logging of all the states and
    % time
    for jj = 1:ADA_N_instances
        sensorTot.ada.data{jj}.xp(sensorTot.ada.n_old:sensorTot.ada.n_old + length(t_ada)-2,:) = sensorData.ada{jj}.xp(2:end, :);
        sensorTot.ada.data{jj}.xv(sensorTot.ada.n_old:sensorTot.ada.n_old + length(t_ada)-2,:) = sensorData.ada{jj}.xv(2:end, :);
    end
    sensorTot.ada.flagApogee(sensorTot.ada.n_old:sensorTot.ada.n_old + length(t_ada)-2, :) = flagApogeeMat(2:end, :);
    sensorTot.ada.flagOpenPara(sensorTot.ada.n_old:sensorTot.ada.n_old + length(t_ada)-2, :) = flagOpenParaMat(2:end, :);
    sensorTot.ada.time(sensorTot.ada.n_old:sensorTot.ada.n_old + length(t_ada)-2) = t_ada(2:end);
    sensorTot.ada.n_old = sensorTot.ada.n_old + length(t_ada)-1;

end

function h = getAltitude(p, temp_ref, p_ref)
    a  = 0.0065;
    n  = 9.807/(287.05*a);

    h  = temp_ref / a * (1 - (p / p_ref)^(1/n));
end

function v = getvelocity(p, dpdt, temp_ref, p_ref)
    a  = 0.0065;
    n  = 9.807/(287.05*a);

    v  = -(temp_ref * dpdt * (p / p_ref)^ (1/n)) / (a * n * p);
end