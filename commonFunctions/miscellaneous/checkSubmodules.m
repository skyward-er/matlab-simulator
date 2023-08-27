msaToolkitURL = 'https://git.skywarder.eu/afd/msa/msa-toolkit';
localRepoPath = '../data/msa-toolkit';
% status = checkLastCommit(msaToolkitURL, localRepoPath, pwd);
% submoduleAdvice(status, msaToolkitURL, localRepoPath, pwd);
if ~exist('../data/msa-toolkit/data',"dir")
    answer = input('WARNING! You don''have the msa toolkit installed. Do you want to install it? (y/n)','s');
    if answer == "y" || answer == "yes"
        cloneType = input('Which type of clone do you want to use? (SSH/HTTP) ','s');
        branchName = input('Which branch do you want to checkout? ','s');
        if cloneType =="SSH" || cloneType == "ssh"
            system('git clone "git@git.skywarder.eu:afd/msa/msa-toolkit.git"')
        elseif cloneType == "HTTP" || cloneType == "http"
            system('git clone "https://git.skywarder.eu/afd/msa/msa-toolkit.git" ../data/msa-toolkit')
        else
            fprintf('\n WARNING! Input not valid: aborting')
        return
        end
        cd('../data/msa-toolkit/')
        system('git fetch')
        system(['git checkout ',branchName])
        cd(currentPath)
    end
    fprintf('\n msa-toolkit installed.\nCurrent branch: %s',branchName)
else
    cd('../data/msa-toolkit/')
    system('git fetch')
    system('git pull')
    cd(currentPath)
end

if ~exist('..\commonFunctions\graphics\general-utilities\','dir')
    % clone repo
    answer = input('WARNING! You don''have the exportStandardizedFigure installed. Do you want to install it? (y/n)','s');
    if answer == "y" || answer == "yes"
        
        cloneType = input('Which type of clone do you want to use? (SSH/HTTP) ','s');
        if cloneType =="SSH" || cloneType == "ssh"
            mkdir('..\commonFunctions\graphics\general-utilities\')    
            system('git clone "git@git.skywarder.eu:skyward/general-utilities.git" ..\commonFunctions\graphics\')
        elseif cloneType == "HTTP" || cloneType == "http"
            mkdir('..\commonFunctions\graphics\general-utilities\')
            system('git clone "https://git.skywarder.eu/skyward/general-utilities.git" ..\commonFunctions\graphics\')
        else
            fprintf('\n WARNING! Input not valid: aborting')
        return
        end
    end
else 
    % update (pull)
    cd('..\commonFunctions\graphics\general-utilities\')
    system('git fetch')
    system('git pull')
    cd(currentPath)
end
