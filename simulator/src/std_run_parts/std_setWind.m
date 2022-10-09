%{
set_wind

std_run - wind generation settings

%}

switch settings.windModel 

    case "constant"

        if settings.montecarlo
            uw = settings.wind.uw;
            vw = settings.wind.vw;
            ww = settings.wind.ww;
            Az = settings.wind.Az;
            El = settings.wind.El;
    
            settings.constWind = [uw, vw, ww];
            saveConstWind =  [uw, vw, ww, Az, El];
        else 
            [uw, vw, ww, Az, El] = windConstGenerator(settings.wind);
            settings.constWind = [uw, vw, ww];
            saveConstWind =  [uw, vw, ww, Az, El];
        end

    case "multiplicative"
       
        Mag  = settings.wind.inputGround;
        Az = settings.wind.inputAzimut(1);
        R = Mag*angle2dcm(Az, 0, 0, 'ZYX');
        uw = R(1,1);
        vw = R(1,2);
        ww = R(1,3);
        settings.constWind = [uw, vw, ww];

end