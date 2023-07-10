%{
set_wind

std_run - wind generation settings

%}
function [uw, vw, ww, Az, El, Mag] = std_setWind(settings)

switch settings.windModel 

    case "constant"

        if settings.montecarlo
            uw = settings.wind.uw;
            vw = settings.wind.vw;
            ww = settings.wind.ww;
            Az = settings.wind.Az;
            El = settings.wind.El;
        else 
            [uw, vw, ww, Az, El] = windConstGenerator(settings.wind);       
        end
        
        Mag = sqrt(uw^2 + vw^2 + ww^2);

    case "multiplicative"
       
        Mag  = settings.wind.inputGround;
        Az = settings.wind.inputAzimut(1);
        R = Mag*angle2dcm(Az, 0, 0, 'ZYX');
        uw = R(1,1);
        vw = R(1,2);
        ww = R(1,3); 
        El = acos(ww/Mag);
end