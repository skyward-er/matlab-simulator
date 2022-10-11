%{ 

structCutter()

Author: Marco Marchesi - GNC iptl
        marco.marchesi@skywarder.eu

-       Release: 03/09/2022
%}

function structOut = structCutter(structIn,type,t0,t1,options,freq)

%{
HELP:
use this function to cut out unwanted values from struct arrays

INPUT:
- structIn: struct that you want to trim from timestamp t0 to timestamp t1
- t0: initial trim time
- t1: final trim time
- options: specify some options for the function
    - "resample" to resample a struct with a given frequency
- freq: frequency of resampling
OUTPUT: 
- structOut: trimmed struct 

%}

if nargin < 5
   
    options = [""];
    freq = [];

end
namesStructIn = fieldnames(structIn);


switch type

    case "timestamp"
        % remove first samples
        savingValues1 = structIn.(namesStructIn{1})(structIn.(namesStructIn{1})>t0);
        len1 = length(savingValues1);
        
        % remove last samples
        savingValues2 = savingValues1(savingValues1<t1);
        len2 = len1 - length(savingValues2);
        
        if length(structIn.(namesStructIn{1}))-len1 == 0 || len1 == 0
            warning('OCIO! You set time window values out of the timestamp boundaries. Also check if the timestamp is in seconds or microseconds.')
        end

    case "index"
        
        % remove first samples
        len1 = length(structIn.(namesStructIn{1}))-t0;
        
        % remove last samples
        len2 = length(structIn.(namesStructIn{1}))-t1;

end

% adjust time, then all the others consequently
structOut.(namesStructIn{1}) = structIn.(namesStructIn{1})(end-len1:end-len2);
for i = 2:size(namesStructIn)
    if length(structIn.(namesStructIn{i})) == length(structIn.(namesStructIn{1}))
        structOut.(namesStructIn{i}) = structIn.(namesStructIn{i})(end-len1:end-len2,:);
    else
        structOut.(namesStructIn{i}) = structIn.(namesStructIn{i});
    end
end

if options == "resample"
    
    dt = 1/freq;
    t_vec = [t0:dt:t1];
    structOutParz.(namesStructIn{1}) = t_vec;

    for i = 2:size(namesStructIn)
            structOutParz.(namesStructIn{i}) = interp1(structOut.(namesStructIn{1}),structOut.(namesStructIn{i}),t_vec);
    end
    structOut = structOutParz;

end

end