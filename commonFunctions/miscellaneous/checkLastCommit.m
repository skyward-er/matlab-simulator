function status = checkLastCommit(headUrl, repoPath, actualPath)
%{
checkLastCommit - function to check if in the local path the submodule of
the input repository headUrl is up to date.

INPUTS:
        - headUrl,      string, url of the github repository (MASTER);
        - repoPath,     string, local path of the submodule;
        - actualPath,   string, path of the calling script/function.

OUTPUTS:
        - status,       double [1x1], git status of the submodule.

CALLED FUNCTIONS: -

VERSIONS:
        -0   27/07/2022,   release,    Davide Rosato

Copyright © 2022, Skyward Experimental Rocketry, AFD department
All rights reserved

SPDX-License-Identifier: GPL-3.0-or-later

%}

    %% CHECK FOR INTERNET CONNECTION
    isINT = haveInternet;
    
    if not(isINT)
        status = 2;
        return
    end
    
    %% ELSE CHECK FOR UPDATE
    cd(repoPath)
    [~, string] = system("git status");
    actualCommit = splitlines(extractAfter(string, "HEAD detached at "));
    
    % 7 digits code of the actual commit
    actualCommit = actualCommit{1};
    
    str = urlread(headUrl);
    href = extractAfter(headUrl, 'https://github.com/');
    str1 = extractAfter(str, strcat('href="/', href, '/commit/'));
    if isempty(str1)
        str1 = extractAfter(str, strcat('href="/', href, '/tree/'));
    end
    
    str2 = extractBefore(str1, '">');
    
    % 7 digits code of the last commit of the wanted repository
    wantedCommit = str2(1:7);
    
    if strcmp(actualCommit, wantedCommit)
        status = 1;
    else
        status = 0;
    end
    cd(actualPath);
    
end

function tf = haveInternet()
  try
    java.net.InetAddress.getByName('www.google.de');
    tf = true;
  catch
      tf = false;
  end
end