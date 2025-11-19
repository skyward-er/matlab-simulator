function [x, P] = NAS_step( x_prev, P_prev, dt_k, measures, settings, environment)
%% Declare variables %%%%%%%%%%%%
xlin    = x_prev(1:6);
xq      = x_prev(7:end);
Plin    = P_prev(1:6, 1:6);
Pq      = P_prev(7:end, 7:end);
nas     = settings.nas;

%% Prediction %%%%%%%%%%%%
if nas.flag.flag_linear_propagation
    [xlin, ~, Plin] = predictorLinear2(xlin, Plin, dt_k, measures.acc, ...
                                    xq(1:4), nas.QLinear);
end

if nas.flag.flag_attitude_propagation
    [xq, Pq] = predictorQuat(xq, Pq, measures.gyro, dt_k, ...
                             nas.Qq);
end

%% Correction %%%%%%%%%%%%
if check_gps_correction(nas, measures)
    [fix, ~] =   gpsFix(measures.acc);
    [xlin, Plin,~] = correctionGPS(xlin, Plin, ...
                        measures.gps.pos, measures.gps.vel, nas.sigma_GPS,...
                        fix, environment.lat0, environment.lon0, nas.GPS.a, nas.GPS.b);
end

if nas.flag.flag_baro_correction
    [xlin, Plin,~]     = correctionBarometer(xlin, Plin, measures.baro, ...
                                             nas, environment);
end

if check_pitot_correction(nas, measures, settings)
    P_c = [Plin, zeros(6, 6); zeros(6, 6), Pq];
    [xlin, Plin] = correctionPitotQuat([xlin, xq], P_c, ...
                    measures.pitot.dyn_press, measures.pitot.stat_press, ...
                    nas.sigma_pitot_static, ...
                    nas.sigma_pitot_dynamic, ...
                    nas.baro, environment);

end

% Here add the eventual check for the apogee, need to modify signature

%% Assemble variables %%%%%%%%%%%%
x = [xlin, xq];
P = [      Plin, zeros(6,6); 
     zeros(6,6),         Pq];
end

