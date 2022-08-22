function [flagMotorShutdown] = run_simulated_motorShutdown(z,V_norm,m,g,settings,contSettings,ap)
%{

decide when to switch off the engine (only for Hybrid/Liquid engines)

CALLED FUNCTIONS: 
extension_From_Angle, getDrag, atmosisa (aero toolbox)

REVISIONS:
- 0     23/08/2022, Release,    Marco Marchesi, SCS department
                                marco.marchesi@skywarder.eu


Copyright © 2022, Skyward Experimental Rocketry, SCS department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

[~,~,~, rho] = atmosisa(z);
CD = getDrag(V_norm,z,extension_From_Angle(ap,settings),contSettings.coeff_Cd);
B = 0.5 * CD * rho * settings.S / m;
h_f = z + 1/(2 * B) * log(1 + V_norm^2 * B / g); % magic formula

if h_f > settings.z_final
    flagMotorShutdown = true;
else
    flagMotorShutdown = false;
end