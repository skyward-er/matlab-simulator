function [xp, P, xv, lastBaroTimestamp] = run_ADA_update(t_ada, dt, ada_settings, xp_prev, P_prev, baro_data, lastBaroTimestamp, getAltitude, getVelocity)

    % Define state matrices
    At = [1     dt      0.5*dt^2;
          0     1       dt;
          0     0       1];
    Ct = [1     0       0];    

    % Prediction step:
    xp = (At * xp_prev')';

    % Prediction variance propagation:
    P = ada_settings.Q + At * P_prev * At';

    % Check if a new barometer measurement is available
    t_baro = baro_data.time;
    idx_baro = sum(t_ada >= t_baro);
    if t_baro(idx_baro) > lastBaroTimestamp
        % Perform the correction step ONLY IF a new barometer measurement
        % is available
        S = Ct * P * Ct' + ada_settings.R;
        K = P * Ct' / S;
        xp = (xp' + K*(baro_data.measures(idx_baro) - Ct*xp'))';
        P = (eye(3) - K*Ct) * P;        
        
        % Update the last used barometer timestamp
        lastBaroTimestamp = t_baro(idx_baro);
    end

    % Convert the pressure state in altitude and velocity
    xv(1) = getAltitude(xp(1), ada_settings.temp_ref, ada_settings.p_ref);
    xv(2) = getVelocity(xp(1), xp(2), ada_settings.temp_ref, ada_settings.p_ref);

end