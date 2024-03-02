function [current_detection, y_t] = linear_simulation_mean(Mdl1, Mdl2, start_detection_ts, stop_detection_ts, apogee_ts, window_size, timestamps, y) % this function generates labels in a more realistic way, since it uses data after having collected it
    %INIT
    last_detection = zeros(1,3);
    apogee_surpassed = false;
    y_t = zeros(length(y()), 1);
    current_detection = zeros(length(y()), 3);

    for t = 1:length(y()) %simulates the time passing
        if t == apogee_ts
            apogee_surpassed = true;
        end
        if(t > start_detection_ts && t < stop_detection_ts) % this condition checks if we are past liftoff and before parafoil deployment? (i'm not sure its that tbh)
            %testing if the current window has any fault detection
            if(last_detection(1) == false)
                [current_detection(t, 1)] = fault_detection_reduced_fft(Mdl1, Mdl2, apogee_surpassed, window_size, timestamps(:, 1), y(:, 1), t); %this function takes a chunk of data which goes backwards in time of a window_size
            end
            if(last_detection(2) == false)
                [current_detection(t, 2)] = fault_detection_reduced_fft(Mdl1, Mdl2, apogee_surpassed, window_size, timestamps(:, 2), y(:, 2), t);
            end
            if(last_detection(3) == false)
                [current_detection(t, 3)] = fault_detection_reduced_fft(Mdl1, Mdl2, apogee_surpassed, window_size, timestamps(:, 3), y(:, 3), t);
            end
            %testing if there are any previous exlusions and combine them to
            %the current needed exclusions
            if last_detection(1) || current_detection(t, 1) == 1
                current_detection(t, 1) = 1;
            end
            if last_detection(2) || current_detection(t, 2) == 1
                current_detection(t, 2) = 1;
            end
            if last_detection(3) || current_detection(t, 3) == 1
                current_detection(t, 3) = 1;
            end
            
            
            
            %checking if all sensors have faults, in this case i reset the
            %values of the R vector in the hope that having all sensors running
            %the eventual faults get mellowed out by the three sensors running
            %in unison
            n = 0;
            if(sum(current_detection(t, :)) == 3 || sum(current_detection(t, :)) == 0) %should we make it indefinite with a boolean var?
                %mean of all sensors
                y_t(t,1) = (y(t, 1) + y(t, 2) + y(t, 3))/3;
                last_detection(1,:) = zeros(1, 3);
            else
                last_detection(1,:) = current_detection(t, :);
                if(current_detection(t, 1) == false)
                    n = y(t, 1) + n;
                end
                if(current_detection(t, 2) == false)
                    n = y(t, 2) + n;
                end
                if(current_detection(t, 3) == false)
                    n = y(t, 3) + n;
                end
                if(sum(current_detection(t, :)) == 1) % one sensor is faulty, mean must me done on the rest
                    y_t(t,1) = n/2;
                else % only one sensor is good
                    y_t(t,1) = n;
                end
            end
            
        elseif(t <= start_detection_ts)
            current_detection(t, :) = zeros(1, 3);
            y_t(t,1) = (y(t, 1) + y(t, 2) + y(t, 3))/3;
        else
            n = 0;
            if(sum(last_detection(1,:)) == 3 || sum(last_detection(1,:)) == 0) %should we make it indefinite with a boolean var?
                %mean of all sensors
                y_t(t,1) = (y(t, 1) + y(t, 2) + y(t, 3))/3;
            else
                if(last_detection(1,1) == false)
                    n = y(t, 1) + n;
                end
                if(last_detection(1,2) == false)
                    n = y(t, 2) + n;
                end
                if(last_detection(1,3) == false)
                    n = y(t, 3) + n;
                end
                if(sum(last_detection(1,:)) == 1) % one sensor is faulty, mean must me done on the rest
                    y_t(t,1) = n/2;
                else % only one sensor is good
                    y_t(t,1) = n;
                end
            end
        end
    end
end
