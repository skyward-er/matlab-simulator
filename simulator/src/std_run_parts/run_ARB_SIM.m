function [ap_ref_new,contSettings,varargout] = run_ARB_SIM(sensorData,settings,contSettings,ap_ref_old)


%% HELP:
%{
air brakes strategy chooser

INPUT: 

- nas_state: state of the rocket, contains velocities and position, can
  come from the actual NAS, or from the simulated exact values, depending
  if NAS actually is working or not
- settings, contSettings: structs that contain all informations about
  mission, rocket, etc, and control settings
- ap_ref_old: old value of angle reference, needed because of how filters
  work

OUTPUT:

- ap_ref_new: new reference angle value
- contSettings: update of contSettings state
- varargout - only if PID_2021 strategy is used:
      {1} = vz_setpoint; % contains the setpoint trajectories
      {2} = z_setpoint;
      {3} = input_output_test; % contains information for cpp algorithms

%}

switch contSettings.algorithm % set this value in configControl.m

    case 'interp'
        % interpolation algorithm: takes two references (max and
        % min extension) and decides how much to open with an
        % interpolation at fixed altitude of the actual velocity
        % w.r.t. the two references.

        [ap_base_filter] = control_Interp(sensorData.kalman.z-settings.z0,sensorData.kalman.vz,contSettings.reference.Z,contSettings.reference.Vz,contSettings.interpType,contSettings.N_forward,settings,contSettings); % cambiare nome alla funzione tra le altre cose

        % filter control action
        if contSettings.flagFirstControl == false % the first reference is given the fastest possible (unfiltered), then filter
            ap_ref_new = ap_ref_old + (ap_base_filter - ap_ref_old)*contSettings.filter_coeff;
        else
            ap_ref_new = ap_base_filter;
        end
        contSettings.flagFirstControl = false;
        if sensorData.kalman.time>contSettings.Tfilter
            contSettings.Tfilter = contSettings.Tfilter+contSettings.deltaTfilter;
            contSettings.filter_coeff = contSettings.filter_coeff/contSettings.filterRatio;
        end

       
    case 'shooting'
        % shooting algorithm:

        [ap_ref_new] = control_Interp(sensorData.kalman.z-settings.z0,sensorData.kalman.vz,contSettings.reference.Z,contSettings.reference.Vz,'linear',contSettings.N_forward,settings); % cambiare nome alla funzione tra le altre cose
        init.options = optimoptions("lsqnonlin","Display","off");
        
        if not(contSettings.flagFilter)
            [ap_ref_new] = control_Shooting([-sensorData.kalman.z-settings.z0,sensorData.kalman.vz],ap_ref_new,settings,contSettings.coeff_Cd,settings.arb,init);
        else
            [ap_base_filter] = control_Shooting([-sensorData.kalman.z-settings.z0,sensorData.kalman.vz],ap_ref_new,settings,contSettings.coeff_Cd,settings.arb,init);

            % filter control action
            if contSettings.flagFirstControl == false % the first reference is given the fastest possible (unfiltered), then filter
                ap_ref_new = ap_ref_old + (ap_base_filter - ap_ref_old)*contSettings.filter_coeff;
            else
                ap_ref_new = ap_base_filter;
            end
            contSettings.flagFirstControl = false;
            if sensorData.kalman.time>contSettings.Tfilter
                contSettings.Tfilter = contSettings.Tfilter+contSettings.deltaTfilter;
                contSettings.filter_coeff = contSettings.filter_coeff/contSettings.filterRatio;
            end

        end

    case 'PID_2021'
        % PID algorithm: at first checks speed and altitude to find
        % which of the tabulated references is the closest, then
        % tries to follow it with a PI controller.

        
        V_norm = norm([sensorData.kalman.vx, sensorData.kalman.vy, sensorData.kalman.vz]);
        % still don't know what these are, but they are useful (at
        % least it seems):
        zc    =    exp_mean(-sensorData.kalman.x_c(:,3),0.8);
        vzc   =    exp_mean(-sensorData.kalman.x_c(:,6),0.8);
        vc    =    exp_mean(sqrt(sensorData.kalman.x_c(:,4).^2+sensorData.kalman.x_c(:,5).^2+sensorData.kalman.x_c(:,6).^2),0.8);

%         [alpha_degree, vz_setpoint, z_setpoint, pid,U_linear, Cdd, delta_S, contSettings] =   control_PID     (nas_state, V_norm,  contSettings,settings);
        [alpha_degree, vz_setpoint, z_setpoint, contSettings] =   control_PID     (sensorData.kalman, V_norm,  contSettings,settings);

        ap_ref_new = deg2rad(alpha_degree);

        input_output_test = struct('alpha_degree', alpha_degree, 'vz_setpoint', vz_setpoint, 'z_setpoint', z_setpoint, 'z', zc, 'vz', vzc, 'Vmod', V_norm);
        contSettings.indice_test = contSettings.indice_test +1;

%      alpha_degree_old = alpha_degree;
        varargout{1} = vz_setpoint;
        varargout{2} = z_setpoint;
        varargout{3} = input_output_test;

    case 'PID_2refs'
        % PID 2 references algorithm: this is a generalization of the
        % 'interp' algorithm formally defined as a controller (PI)
        % with two reference states and one output fed back.
        % Basically takes two references that define an interval instead
        % of one that only gives a point and tries to keep the state
        % inside that interval

        if not(contSettings.flagFilter)

            [ap_ref_new] = control_PID2refs(z,vz,settings.reference.Z,settings.reference.Vz,contSettings.N_forward,settings,contSettings); % cambiare nome alla funzione tra le altre cose

        else
 
            [ap_base_filter] = control_PID2refs(sensorData.kalman.z,sensorData.kalman.vz,settings.reference.Z,settings.reference.Vz,contSettings.N_forward,settings,contSettings); % cambiare nome alla funzione tra le altre cose

            % filter control action
            if contSettings.flagFirstControl == false % the first reference is given the fastest possible (unfiltered), then filter
                ap_ref_new = ap_ref_old + (ap_base_filter -ap_ref_old)*contSettings.filter_coeff;
            else
                ap_ref_new = ap_base_filter;
            end
            
            if sensorData.kalman.time>contSettings.Tfilter
                contSettings.Tfilter = contSettings.Tfilter+contSettings.deltaTfilter;
                contSettings.filter_coeff = contSettings.filter_coeff/contSettings.filterRatio;
            end

        end

end

contSettings.flagFirstControl = false;


