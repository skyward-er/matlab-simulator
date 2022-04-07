function [value, isterminal, direction] = eventBurnOff(t, ~, settings, varargin)
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

value = settings.tb - t;

isterminal = 1;
direction = 0;


end

