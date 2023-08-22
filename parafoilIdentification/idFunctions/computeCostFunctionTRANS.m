% computeCostFunctionTRANS
%
% HELP:
% 
% Computes the cost function for the identification of the parameters for
% parafoil
%
% The "TRANS" version aims to estimate all the
% 10  aerodynamic coefficients related to the translational dof of the parafoil
function [outJ] = computeCostFunctionTRANS(x, t_m, y_m, R_m, settings, contSettings,deltaA)
    % Compute cost function for the parameter estimation.
    % Inputs
    %   - x:   Coefficients
    %   - y_m: 3D matrix with test results (i, j, k)
    %             - i is the test number
    %             - j is the sample number
    %             - k is the state number
    %                  -1-3:    position (inertial frame)
    %                  -4-6:    quaternions
    %                  -7-9:   velocity (body frame)
    %                  -10-12:  angular velocity (body frame)
    %                  -13-14:  input sig (control)
    %   - R_m: Inverse of covariance matrix R^-1
    % Output
    %   - J:   Cost function used for the optimization
      
    % Initialize parameters
    
    settings.payload.CD0       =  x(1); 
    settings.payload.CDAlpha2  =  x(2);
    settings.payload.CL0       =  x(3);
    settings.payload.CLAlpha   =  x(4);
    settings.payload.Cm0       =  x(5);
    settings.payload.CmAlpha   =  x(6);
    settings.payload.Cmq       =  x(7);
    settings.payload.Cnr       =  x(8);
    settings.payload.Clp       =  x(9);
    settings.payload.ClPhi     =  x(10);

    % settings.payload.deltaSMax =  x(15);

    % Initialize cost
    J = zeros(size(R_m));
    
     
    % Run simulation
    % recall initial state
    % ode wants NED, VBODY, p,q,r, qw, qx, qy, qz, deltaA as states, while nas retrieves only NED, VNED, qx,qy,qz,qw
    Y0 = [y_m(1,1:6), zeros(1,3), [y_m(1,10), y_m(1,7:9)],0]; 

    % rotate velocities in body frame
    Y0(1,4:6) = quatrotate(Y0(1,10:13),Y0(1,4:6));

    [t_sim, y_sim] = callSimulator(deltaA, settings,contSettings,t_m,Y0 );
    
    % Interpolation of simulation - teoricamente inutile se usiamo t_m come
    % vettore per la ode
    y_sim = interp1(t_sim, y_sim, t_m);

    % Compute difference between both
    diff(:,1:6) = y_m(:,1:6) - y_sim(:,1:6);
    

    % Cost computation
    J = J + (R_m*diff')*diff;       
    outJ = norm(J);
end