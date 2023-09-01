% computeCostFunctionFULL
%
% HELP:
% 
% Computes the cost function for the identification of the parameters for
% parafoil
%
% The "FULL" version uses all the dof for the computation and aims to estimate all the
% 14  aerodynamic coefficients of the parafoil
function [outJ] = computeCostFunction(x, t_m, y_m, R_m, settings, contSettings,ABK)
    % Compute cost function for the parameter estimation.
    % Inputs
    %   - x:   Coefficients
    %   - y_m: nas measurements
    %   - R_m: Inverse of covariance matrix R^-1
    % Output
    %   - J:   Cost function used for the optimization
      
    % Initialize parameters
    
    settings.CD_correction = x;

    % Initialize cost
    J = zeros(size(R_m));
    
     
    % Run simulation
    % recall initial state
    % ode wants NED, VBODY, p,q,r, qw, qx, qy, qz, deltaA as states, while nas retrieves only NED, VNED, qx,qy,qz,qw
    Y0 = [y_m(1,1:6), zeros(1,3), [y_m(1,10), y_m(1,7:9)],0]; 

    % rotate velocities in body frame
    Y0(1,4:6) = quatrotate(Y0(1,10:13),Y0(1,4:6));

    [t_sim, y_sim] = callSimulatorAscent(ABK, settings,contSettings,t_m,Y0 );
    
    % Interpolation of simulation - teoricamente inutile se usiamo t_m come
    % vettore per la ode
    y_sim = interp1(t_sim, y_sim, t_m);

    % Compute difference between both
    diff(:,1:6) = y_m(:,1:6) - y_sim(:,1:6);    

    % Cost computation
    J = J + (R_m*diff')*diff;       
    outJ = norm(J);
end