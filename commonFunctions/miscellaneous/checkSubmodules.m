msaToolkitURL = 'https://git.skywarder.eu/afd/msa/msa-toolkit';
localRepoPath_msaToolkit = '../data/msa-toolkit';
% status = checkLastCommit(msaToolkitURL, localRepoPath, pwd);
% submoduleAdvice(status, msaToolkitURL, localRepoPath, pwd);
if ~exist('../data/msa-toolkit/data',"dir")
    answer = input('WARNING! You don''have the msa toolkit installed. Do you want to install it? (mandatory) (y/n)','s');
    if answer == "y" || answer == "yes"
        cloneType = input('Which type of clone do you want to use? (SSH/HTTP) ','s');
        branchName = input('Which branch do you want to checkout? ','s');
        if cloneType =="SSH" || cloneType == "ssh"
            system(['git clone "git@git.skywarder.eu:afd/msa/msa-toolkit.git" ',localRepoPath_msaToolkit])
        elseif cloneType == "HTTP" || cloneType == "http"
            system(['git clone "https://git.skywarder.eu/afd/msa/msa-toolkit.git" ',localRepoPath_msaToolkit])
        else
            fprintf('\n WARNING! Input not valid: aborting')
        return
        end
        cd(localRepoPath_msaToolkit)
        system('git fetch')
        system(['git checkout ',branchName])
        cd(currentPath)
    end
    fprintf('\n msa-toolkit installed.\nCurrent branch: %s',branchName)
else
    cd(localRepoPath_msaToolkit)
    system('git fetch')
    system('git pull')
    cd(currentPath)
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
    % update (pull)
    cd(localRepoPath_generalUtilities)
    system('git fetch')
    system('git pull')
    cd(currentPath)
end
