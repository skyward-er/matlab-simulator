function [t_est_tot, x_est_tot, xp_ada_tot, xv_ada_tot, t_ada_tot, nas, P_ada, P_c] = runControl(settings, nas, sensorData, tot, ada_prev, Pada_prev, flagAeroBrakes)
    %% ADA
    if settings.Ada && settings.dataNoise
        [xp_ada, xv_ada, P_ada, settings.ada]   =  run_ADA(ada_prev, Pada_prev, sensorData, settings.ada);
    
        xp_ada_tot(tot.n_ada_old:tot.n_ada_old + size(xp_ada(:,1),1) -1,:)  = xp_ada(1:end,:);
        xv_ada_tot(tot.n_ada_old:tot.n_ada_old + size(xv_ada(:,1),1)-1,:)  = xv_ada(1:end,:);
        t_ada_tot(tot.n_ada_old:tot.n_ada_old + size(xp_ada(:,1),1)-1)     = sensorData.barometer.time;
        tot.n_ada_old = tot.n_ada_old + size(xp_ada,1);
    
    end
    %% Navigation system
    if settings.Kalman && settings.dataNoise
    
        [x_c, P_c, settings.kalman, nas]   =  run_kalman(sensorData, settings.kalman, nas);
    
        x_est_tot(tot.n_est_old:tot.n_est_old + size(x_c(:,1),1)-1,:)  = x_c(1:end,:);
        t_est_tot(tot.n_est_old:tot.n_est_old + size(x_c(:,1),1)-1)    = sensorData.accelerometer.time;
        tot.n_est_old = tot.n_est_old + size(x_c,1);
    
    end
    %% Control algorithm
    
    csett = struct([]); %TODO: fix in merge with new sim

    if flagAeroBrakes && settings.Kalman && settings.control
        zc    =   -x_c(end,3);
        vzc   =   -x_c(end,6);
        vc    =    sqrt(x_c(end,4).^2+x_c(end,5).^2+x_c(end,6).^2);
        if tot.ctr_start == -1
            tot.ctr_start = 0.1*(n - 1);
        end
        %% selection of controler type
        switch csett.flagPID
            case 1
                [alpha_degree, vz_setpoint, z_setpoint, pid, U_linear, Cdd, delta_S, csett] = control_PID    (zc, vzc, vc, csett);
            case 2
                [alpha_degree, vz_setpoint, z_setpoint, pid, U_linear, Cdd, delta_S, csett] = control_Lin    (zc, vzc, vc, csett);
            case 3
                [alpha_degree, vz_setpoint, z_setpoint, csett]                              = control_Servo  (zc, vzc,  csett);
        end
        %              input_output_test(indice_test) = struct('alpha_degree', alpha_degree, 'vz_setpoint', vz_setpoint, 'z_setpoint', z_setpoint, 'z', zc, 'vz', vzc, 'Vmod', sqrt(vxxx^2 + vyyy^2 + vz^2));
        %              indice_test = indice_test +1;
    
        extension = extension_From_Angle(alpha_degree);
    elseif flagAeroBrakes && ~settings.Kalman && settings.control
        if tot.ctr_start == -1
            tot.ctr_start = 0.1*(n - 1);
        end
        switch csett.flagPID
            case 1
                [alpha_degree, vz_setpoint, z_setpoint, pid,U_linear, Cdd, delta_S, csett] =   control_PID     (z, vz, sqrt(vxxx^2 + vyyy^2 + vz^2),  csett);
            case 2
                [alpha_degree, vz_setpoint, z_setpoint, pid,U_linear, Cdd, delta_S, csett] =   control_Lin     (z, vz, sqrt(vxxx^2 + vyyy^2 + vz^2),  csett);
            case 3
                [alpha_degree, vz_setpoint, z_setpoint, csett]                             =   control_Servo   (z, vz(end),  csett);
        end
    
        %             % Salvo input/output per testare algoritmo cpp
        %             input_output_test(indice_test) = struct('alpha_degree', alpha_degree, 'vz_setpoint', vz_setpoint, 'z_setpoint', z_setpoint, 'z', z, 'vz', vz, 'Vmod', sqrt(vxxx^2 + vyyy^2 + vz^2));
        %             indice_test = indice_test +1;
    
        extension = extension_From_Angle(alpha_degree);
    else
        extension = 0;
    end
    
    if settings.control  && flagAeroBrakes
        % Save the values to plot them
        tot.vz_tot(end + 1)    =  vz;
        tot.z_tot(end + 1)     =  z;
        tot.vz_setpoint_tot(end +1)  =  vz_setpoint;
        tot.z_setpoint_tot(end + 1)   =  z_setpoint;
        tot.alpha_degree_tot(end + 1) =  alpha_degree;
        tot.extension_tot(end + 1) = extension;
        if csett.flagPID ~= 3
            tot.Cd_tot(end + 1)    =  Cdd;
            tot.pid_tot(end + 1)   =  pid;
            tot.U_lin_tot(end + 1) =  U_linear;
            tot.dS_tot(end + 1)    =  delta_S;
        end
    end
end