%{

mission configuration script

%}

conf.year = 2024; % choose between: 2021, 2022, 2023, 2024
conf.flight = "pds"; % choose between: "roccaraso", "pds" ( ponte de sor ) , " "

switch conf.year

    case 2021

        switch conf.flight
            
            case "roccaraso"
                mission.name = 'Lynx_Roccaraso_September_2021';
                mission.nameMSA = '2021_Lynx_Roccaraso_September';

            case "pds"
                mission.name = 'Lynx_Portugal_October_2021';
                mission.nameMSA = '2021_Lynx_Portugal_October';
        end

    case 2022
        
        switch conf.flight

            case "roccaraso"
                mission.name = 'Pyxis_Roccaraso_September_2022';
                mission.nameMSA = '2022_Pyxis_Roccaraso_September';

            case "pds"
                mission.name = 'Pyxis_Portugal_October_2022';
                mission.nameMSA = '2022_Pyxis_Portugal_October';
                
        end

    case 2023

        switch conf.flight
            
            case "roccaraso"
                mission.name = 'Gemini_Roccaraso_September_2023';
                mission.nameMSA = '2023_Gemini_Roccaraso_September';
            
            case "pds"
                mission.name = 'Gemini_Portugal_October_2023';
                mission.nameMSA = '2023_Gemini_Portugal_October';

        end
        
    case 2024

        switch conf.flight
            
            case "roccaraso"
                mission.name = 'Lyra_Roccaraso_September_2024';
                mission.nameMSA = '2024_Lyra_Roccaraso_September';
            
            case "pds"
                mission.name = 'Lyra_Portugal_October_2024';
                mission.nameMSA = '2024_Lyra_Portugal_October';

        end
end
