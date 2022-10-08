%{

mission configuration script

%}

conf.year = 2022; % choose between: 2021, 2022, 2023
conf.flight = "pds"; % choose between: "roccaraso", "pds" ( ponte de sor ) , " "

switch conf.year

    case 2021

        switch conf.flight
            
            case "roccaraso"
                settings.mission = 'Lynx_Roccaraso_September_2021';

            case "pds"
                settings.mission = 'Lynx_Portugal_October_2021';
        end

    case 2022
        
        switch conf.flight

            case "roccaraso"
                settings.mission = 'Pyxis_Roccaraso_September_2022';

            case "pds"
                settings.mission = 'Pyxis_Portugal_October_2022';
                
        end

    case 2023

        switch conf.flight
            
            case "roccaraso"
                settings.mission = 'Gemini_Roccaraso_September_2023';
            
            case "pds"
                settings.mission = 'Gemini_Portugal_October_2023';

        end
end
