    if iTimes==1 && settings.Ada
        ada_prev  =   settings.ada.x0;
        Pada_prev =   settings.ada.P0;
    elseif iTimes ~= 1 && settings.Ada
        ada_prev  =   xp_ada_tot(end,:);
        Pada_prev =   P_ada(:,:,end);
    end
   
    if iTimes==1 && settings.Kalman
        x_prev    =  [X0; V0; Q0(2:4); Q0(1);0;0;0];
        vels_prev =  [0;0;0];
        nas = nasSys(x_prev);
        nas.latitude0 = settings.kalman.lat0;
        nas.longitude0 = settings.kalman.lon0;
        nas.altitude0 = - settings.kalman.z0;
    end
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
        vels_tot(tot.n_est_old:tot.n_est_old + size(x_c(:,1),1)-1,:)  = x_c(1:end,4:6);
        t_est_tot(tot.n_est_old:tot.n_est_old + size(x_c(:,1),1)-1)    = sensorData.accelerometer.time;              
        tot.n_est_old = tot.n_est_old + size(x_c,1); 
     
    end
%% Control algorithm

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
         input_output_test(indice_test) = struct('alpha_degree', alpha_degree, 'vz_setpoint', vz_setpoint, 'z_setpoint', z_setpoint, 'z', zc, 'vz', vzc, 'Vmod', sqrt(vxxx^2 + vyyy^2 + vz^2));
         indice_test = indice_test +1;
         
         x = extension_From_Angle(alpha_degree);
         i = i + 1; 
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
        
        % Salvo input/output per testare algoritmo cpp
        input_output_test(indice_test) = struct('alpha_degree', alpha_degree, 'vz_setpoint', vz_setpoint, 'z_setpoint', z_setpoint, 'z', z, 'vz', vz, 'Vmod', sqrt(vxxx^2 + vyyy^2 + vz^2));
        indice_test = indice_test +1;
         
         x = extension_From_Angle(alpha_degree);
         i = i + 1; 
    else 
        x = 0;
    end    
    
    if settings.control == true  && flagAeroBrakes == 1    
         % Save the values to plot them
         tot.vz_tot(i)    =  vz;
         tot.z_tot(i)     =  z;
         tot.vz_setpoint_tot(i)  =  vz_setpoint;
         tot.z_setpoint_tot(i)   =  z_setpoint;
         tot.alpha_degree_tot(i) =  alpha_degree;
         if csett.flagPID ~= 3
             tot.Cd_tot(i)    =  Cdd;
             tot.pid_tot(i)   =  pid;
             tot.U_lin_tot(i) =  U_linear;
             tot.dS_tot(i)    =  delta_S;
         end
    end
