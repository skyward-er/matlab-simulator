%{

mission configuration script

%}

conf.year = 2022; % choose between: 2021, 2022, 2023, 2024
conf.flight = "pds"; % choose between: "roccaraso", "pds" ( ponte de sor ) , " "

switch conf.year

    case 2021

        switch conf.flight
            
            case "roccaraso"
                settings.mission = 'Lynx_Roccaraso_September_2021';
                settings.missionMSA = '2021_Lynx_Roccaraso_September';

            case "pds"
                settings.mission = 'Lynx_Portugal_October_2021';
                settings.missionMSA = '2021_Lynx_Portugal_October';
        end

    case 2022
        
        switch conf.flight

            case "roccaraso"
                settings.mission = 'Pyxis_Roccaraso_September_2022';
                settings.missionMSA = '2022_Pyxis_Roccaraso_September';

            case "pds"
                settings.mission = '2022_Pyxis_Portugal_October';
                
        end

    case 2023

        switch conf.flight
            
            case "roccaraso"
                settings.mission = '2023_Gemini_Roccaraso_September';
            
            case "pds"
                settings.mission = '2023_Gemini_Portugal_October';

        end
        
    case 2024

        switch conf.flight
            
            case "roccaraso"
                settings.mission = 'Lyra_Roccaraso_September_2024';
                settings.missionMSA = '2024_Lyra_Roccaraso_September';
            
            case "pds"
                settings.mission = 'Lyra_Portugal_October_2024';
                settings.missionMSA = '2024_Lyra_Portugal_October';

        end
end
