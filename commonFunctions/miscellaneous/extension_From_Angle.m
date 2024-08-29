function [x,varargout] = extension_From_Angle(alpha, settings, mission)
% HELP
% alpha must be already in radiants
%
% input:
% alpha: servo angle reference
% settings: a struct containing everything basically

switch mission.name
    
    case '2024_Lyra_Roccaraso_September'
        x = settings.arb.extPol(1)*alpha.^4 + settings.arb.extPol(2)*alpha.^3+settings.arb.extPol(3)*alpha.^2 + settings.arb.extPol(4).*alpha;
        deltaS = alpha * settings.arb.surfPol;
        varargout{1} = deltaS;

    case '2024_Lyra_Portugal_October' % to be modified when updated data are available
        
        x = settings.arb.extPol(1)*alpha.^4 + settings.arb.extPol(2)*alpha.^3+settings.arb.extPol(3)*alpha.^2 + settings.arb.extPol(4).*alpha;
        deltaS = alpha * settings.arb.surfPol;
        varargout{1} = deltaS;

    case 'Gemini_Roccaraso_September_2023'
        x = settings.arb.extPol(1)*alpha.^4 + settings.arb.extPol(2)*alpha.^3+settings.arb.extPol(3)*alpha.^2 + settings.arb.extPol(4).*alpha;
        deltaS = alpha * settings.arb.surfPol;
        varargout{1} = deltaS;

    case '2023_Gemini_Portugal_October' % to be modified when updated data are available
        
        x = settings.arb.extPol(1)*alpha.^4 + settings.arb.extPol(2)*alpha.^3+settings.arb.extPol(3)*alpha.^2 + settings.arb.extPol(4).*alpha;
        deltaS = alpha * settings.arb.surfPol;
        varargout{1} = deltaS;

    case 'Pyxis_Portugal_October_2022'

        x = settings.arb.extPol(1)*alpha.^4 + settings.arb.extPol(2)*alpha.^3+settings.arb.extPol(3)*alpha.^2 + settings.arb.extPol(4).*alpha;
        deltaS = alpha * settings.arb.surfPol;
        varargout{1} = deltaS;
    case 'Pyxis_Roccaraso_September_2022'

        x = settings.arb.extPol(1)*alpha.^4 + settings.arb.extPol(2)*alpha.^3+settings.arb.extPol(3)*alpha.^2 + settings.arb.extPol(4).*alpha;
        deltaS = alpha * settings.arb.surfPol;
        varargout{1} = deltaS;
    
    case 'Lynx_Portugal_October_2021'

        % Obtain delta_S given the servomotor angle
        a = -9.43386/1000;
        b = 19.86779/1000;
        deltaS = a*(alpha^2) + b*alpha;
        varargout{1} = deltaS;
        % delta_S [m^2] = (-9.43386 * alpha^2 + 19.86779 * alpha) * 10^(-3), with alpha in [rad] from 0 to 0.89 rad.
        
        % Obtain aerobrakes extension given the delta_S
        a = -1.04034;
        b = 0.30548;
        x = (-b + sqrt(b^2 + 4*a*deltaS)) / (2*a);
        % A [m^2] = -1.04034 * x^2 + 0.30548 * x, with x in [m] from 0 to 0.03866 m;
    
    case 'Lynx_Roccaraso_September_2021'

        % Obtain delta_S given the servomotor angle
        a = -9.43386/1000;
        b = 19.86779/1000;
        deltaS = a*(alpha^2) + b*alpha;
        varargout{1} = deltaS;
        % delta_S [m^2] = (-9.43386 * alpha^2 + 19.86779 * alpha) * 10^(-3), with alpha in [rad] from 0 to 0.89 rad.
        
        % Obtain aerobrakes extension given the delta_S
        a = -1.04034;
        b = 0.30548;
        x = (-b + sqrt(b^2 + 4*a*deltaS)) / (2*a);
        % A [m^2] = -1.04034 * x^2 + 0.30548 * x, with x in [m] from 0 to 0.03866 m;

end