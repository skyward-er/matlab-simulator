function [sensorData, sensorTot, ada, flagApogee, flagOpenPara]   =  run_ADA(sensorData, sensorTot, settings,tf)

% Author: Alessandro Del Duca
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@skywarder.eu
% email: alessandro.delduca@skywarder.eu
% Release date: 01/03/2021

%{
-----------DESCRIPTION OF FUNCTION:------------------
This function simulate the Apogee Detection Algorithm (ADA), the
algorithm is a 3rd order linear kalman filter with this states:
- x(1): Vertical position
- x(2): Vertical velocity
- x(3): Vertical acceleration

The filter model is a constant acceleration point mass, with trial
computation we will end up with this discrete time system:
- p(t+1) = pt + v*dt +0.5*a*dt^2
- v(t+1) = vt + a*dt
- a(t+1) = const
with observation on the first state y(t) = p(t)

INPUTS: 
    - xin:        [1x3] VECTOR OF PREVIOUS VALUES --> p,v,a

    - Pin:        [3x3] MATRIX OF PREVIOUS COVARIANCE OF STATE

    - t_baro:     [1x2] VECTOR OF TIME INSTANTS AT WHICH THE BAROMETER TOOK 
                        SAMPLES INSIDE THE 0.1 INTEGRATION PERIOD 

    - p_baro:     [1x2] PRESSURE MEASUREMENTS FROM THE BAROMETER

    - ada:              STRUCT WITH THE ADA PARAMETERS:

        - Q:          [3x3] COVARIANCE MATRIX OF PROCESS NOISE

        - R:          [1x1] COVARIANCE MATRIX OF OBSERVATION NOISE

        - flag_apo:   BOOLEAN VARIABLE THAT BECOME TRUE AFTER THE APOGEE IS
                      DETECTED count_threshold TIMES

        - t_ada:      TIME OF THE PREDICTED APOGEE 

        - counter:    COUNTER OF THE NUMBER OF CONSECUTIVES APOGEE
                      DETECTED

        - v_thr:      VELOCITY THRESHOLD FOR THE APOGEE DETECTION

        - count_thr:  COUNTER THRESHOLD FOR THE APOGEE DETECTION

      OUTPUTS:

        - xp:        [1x3] FILTERED STATES OF THE ADA 

        - xv:        [1x2] VERTICAL POSITION AND VELOCITY

        - Pout:      [3x3] PROPAGATED COVARIANCE MATRIX FOR EACH OF THE ESTIMATION TIME
                           INSTANTS. 

        - ada

-----------------------------------------------------------------------
%}

ada = settings.ada;
t_ada = sensorTot.ada.time(end):1/settings.frequencies.ADAFrequency:tf;


xp = zeros(length(t_ada),3);
P = zeros(3,3,length(t_ada));
xv = zeros(length(t_ada),2);

xp(1,:)  = sensorData.ada.xp(end,:);
P(:,:,1) = sensorData.ada.P(:,:,end);
xv(1,:) = sensorData.ada.xv(end,:);

if length(t_ada)>1
    % initialize dt
    dt = diff(t_ada(1:2));

    % retrieve sensor timestamps
    t_baro = sensorTot.barometer.time;

    % define state matrices
    At = [  1      dt    0.5*dt^2;
        0       1        dt;
        0       0         1;];

    Ct = [ 1     0     0 ];


    for ii = 2:length(t_ada)

        % Prediction step:
        xp(ii,:)      =   (At * xp(ii-1,:)')';

        % Prediction variance propagation:

        P(:,:,ii)      =   ada.Q + At * P(:,:,ii-1) * At';

        % Correction step:
        S      =   Ct * P(:,:,ii) * Ct' + ada.R;
        K      =   P(:,:,ii) * Ct' /S;
        index_baro  =  sum(t_ada(ii) >= t_baro);
        xp(ii,:)      =   (xp(ii,:)' + K*(sensorTot.barometer.pressure_measures(index_baro) - Ct*xp(ii,:)'))';
        P(:,:,ii)   =  (eye(3) - K*Ct) * P(:,:,ii);

        xv(ii,1)  =   getaltitude(xp(ii,1),ada.temp_ref, ada.p_ref);
        xv(ii,2)  =   getvelocity(xp(ii,1),xp(ii,2),ada.temp_ref, ada.p_ref);

        if ada.flag_apo  == false
            if xv(ii,2) < ada.v_thr && xv(ii,1) > 100
                ada.counter = ada.counter + 1;
            else
                ada.counter = 0;
            end
            if ada.counter >= ada.count_thr
                ada.t_ada = t_ada(ii);
                ada.flag_apo = true;
            end
        end
        
        if strcmp(settings.scenario, 'descent') || ada.flag_apo
            if ada.flagOpenPara == false
                if xv(ii,1) < settings.para(1).z_cut
                    ada.paraCounter = ada.paraCounter+1;
                else
                    ada.paraCounter = 0;
                end
                if ada.paraCounter >= ada.altitude_confidence_thr
                    ada.t_para = t_ada(ii);
                    ada.flagOpenPara = true;
                end
            end
        end
    end
    
flagApogee = ada.flag_apo;
flagOpenPara = ada.flagOpenPara;
    
sensorData.ada.xp = xp;
sensorData.ada.xv = xv;
sensorData.ada.P = P;
sensorData.ada.time = t_ada;

sensorTot.ada.xp(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xp(:,1),1) -2,:) = sensorData.ada.xp(2:end,:);
sensorTot.ada.xv(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xv(:,1),1)-2,:)  = sensorData.ada.xv(2:end,:);
sensorTot.ada.time(sensorTot.ada.n_old:sensorTot.ada.n_old + size(sensorData.ada.xp(:,1),1)-2)  = sensorData.ada.time(2:end);
sensorTot.ada.n_old = sensorTot.ada.n_old + size(sensorData.ada.xp,1)-1;

end


end


function h = getaltitude(p, temp_ref, p_ref)
a  = 0.0065;
n  = 9.807/(287.05*a);

h  = temp_ref / a * (1 - (p / p_ref)^(1/n));
end

function v = getvelocity(p, dpdt, temp_ref, p_ref)
a  = 0.0065;
n  = 9.807/(287.05*a);

v  = -(temp_ref * dpdt * (p / p_ref)^ (1/n)) / (a * n * p);
end

