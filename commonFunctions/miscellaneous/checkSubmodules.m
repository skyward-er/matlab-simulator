answer_pull = input('Do you want to pull the matlab simulator? (y/n)','s');
if answer_pull == "yes" || answer_pull == "y"
    cd('../')
    system('git fetch')
    system('git pull')
    cd(currentPath)
end

msaToolkitURL = 'https://git.skywarder.eu/afd/msa/msa-toolkit';
localRepoPath_msaToolkit = '../data/msa-toolkit';
if ~exist('../data/msa-toolkit/data',"dir")
    answer = input(['WARNING! You don''have the msa toolkit installed.\n' ...
        'Do you want to install it now? ~2Gb of data. (y/n)\n' ...
        'Keep in mind that it is necessary to run the simulator\n' ...
        'BUT a good internet connection is strongly recommended\n'],'s');
    if answer == "y" || answer == "yes"
        cloneType = input('Which type of clone do you want to use? (SSH/HTTP)\n','s');
        branchName = input('Which branch do you want to checkout?\n','s');
        if cloneType =="SSH" || cloneType == "ssh"
            system(['git clone "git@git.skywarder.eu:afd/msa/msa-toolkit.git" ',localRepoPath_msaToolkit])
        elseif cloneType == "HTTP" || cloneType == "http"
            system(['git clone "https://git.skywarder.eu/afd/msa/msa-toolkit.git" ',localRepoPath_msaToolkit])
        else
            fprintf('\n WARNING! Input not valid: aborting\n')
        return
        end
        cd(localRepoPath_msaToolkit)
        system('git fetch')
        system(['git checkout ',branchName])
        cd(currentPath)
    end
    fprintf('\n msa-toolkit installed.\nCurrent branch: %s',branchName)
else
    answer_pull = input('Do you want to pull the msa-toolkit? (y/n)','s');
    if answer_pull == "yes" || answer_pull == "y"
        cd(localRepoPath_msaToolkit)
        system('git fetch')
        system('git pull')
        cd(currentPath)
    end
end

localRepoPath_generalUtilities = '..\commonFunctions\graphics\general-utilities\';
if ~exist([localRepoPath_generalUtilities,'exportStandardizedFigure'],'file')
    % clone repo
    answer = input('WARNING! You don''have the exportStandardizedFigure installed. Do you want to install it? (recommended) (y/n)','s');
    if answer == "y" || answer == "yes"
        
        cloneType = input('Which type of clone do you want to use? (SSH/HTTP) ','s');
        if cloneType =="SSH" || cloneType == "ssh"
            mkdir(localRepoPath_generalUtilities)    
            system(['git clone "git@git.skywarder.eu:skyward/general-utilities.git" ',localRepoPath_generalUtilities])
        elseif cloneType == "HTTP" || cloneType == "http"
            mkdir(localRepoPath_generalUtilities)
            system(['git clone "https://git.skywarder.eu/skyward/general-utilities.git" ',localRepoPath_generalUtilities])
        else
            fprintf('\n WARNING! Input not valid: aborting')
        return
        end
    end
else 
    answer_pull = input('Do you want to pull the graphics? (y/n)','s');
    if answer_pull == "yes" || answer_pull == "y"
        cd(localRepoPath_generalUtilities)
        system('git fetch')
        system('git pull')
        cd(currentPath)
    end
end
