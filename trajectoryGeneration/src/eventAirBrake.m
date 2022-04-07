function [value, isterminal, direction] = eventAirBrake(t, Y, settings, varargin)
%{
eventApogee - Event function to stop simulation at burn out

INPUTS:     
        - t, double [n° variations, 1], integration time, [s];
        - Y, double [n° variations, 16], state matrix,
                            [ x y z | u v w | p q r | q0 q1 q2 q3 | m | Ixx Iyy Izz ]:
                            * (x y z), NED{north, east, down} horizontal frame;
                            * (u v w), body frame velocities;
                            * (p q r), body frame angular rates;
                            * m , total mass;
                            * (Ixx Iyy Izz), Inertias;
                            * (q0 q1 q2 q3), attitude unit quaternion.
        - settings, struct, rocket and simulation data.

OUTPUTS:        
        - value, selected value to check if the integration has to be stopped (vertical velocity);
        - isterminal, logical input to stop the integration;
        - direction, to select the sign that the function must have when tends to zero, 1 = positive.

CALLED FUNCTIONS: -

REVISIONS:
- #0    07/04/2022, Release, Davide Rosato
Copyright © 2021, Skyward Experimental Rocketry, AFD department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

if t > settings.tb
    
    %% RECALL THE STATE
    z = -Y(3);
    u = Y(4);
    v = Y(5);
    w = Y(6);
    q0 = Y(10);
    q1 = Y(11);
    q2 = Y(12);
    q3 = Y(13);
    
    %% QUATERION ATTITUDE
    Q = [q0 q1 q2 q3];
    Q = Q/norm(Q);
    
    %% WIND (supposed constant)
    uw = settings.constWind(1); vw = settings.constWind(2); ww = settings.constWind(3);
    dcm = quatToDcm(Q);
    wind = dcm*[uw; vw; ww];
   
    %% RELATIVE VELOCITIES (plus wind);
    ur = u - wind(1);
    vr = v - wind(2);
    wr = w - wind(3);

    V_norm = norm([ur vr wr]);
    Mach = getMach(V_norm, z + settings.z0);
    
    value = Mach - settings.MachControl;
    
else
    value = 1;
end


isterminal = 1;
direction = 0;


end

