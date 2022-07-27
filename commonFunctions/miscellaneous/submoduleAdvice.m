function [] = submoduleAdvice(status, repoURL, localPath, actualPath)
%{
submoduleAdvice - function to display a questlog to ask the user if he
wants to update the submodule

INPUTS:
        - status,   double [1x1],   git status of the submodule;
        - repoURL,  string,         URL of the submodule repository.

OUTPUTS:    /

CALLED FUNCTIONS: -

VERSIONS:
        -0   27/07/2022,   release,    Davide Rosato

Copyright © 2022, Skyward Experimental Rocketry, AFD department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

repoName = extractAfter(repoURL, 'github.com/');

% Skip if the repository is updated
if status == 1
    
    return
    
elseif status == 2
    
    answer = questdlg({"!! INTERNET CONNECTION MISSING !!", ...
        strcat("Submodule repository ", repoName, "  may not be updated!"), "  ", ...
            'Do you want to continue anyway or abort running?'}, ...
            'Dialog', 'Continue', 'Abort', 'Abort');
else

    % Else ..
    answer = questdlg({"!! WARNING !!", ...
        strcat("Submodule repository ", repoName, " is not updated!"), "  ", ...
            'Do you want to update new commits of the repository or continue anyway?'}, ...
            'Dialog', 'Update', 'Continue', 'Abort', 'Abort');

end

if strcmp(answer, 'Abort')
    error('Simulation aborted')
elseif strcmp(answer, 'Update')
    fprintf('Updating submodule ... \n\n')
    cd(localPath)
    eval(strcat("!git pull ", repoURL));
    cd(actualPath)

end

