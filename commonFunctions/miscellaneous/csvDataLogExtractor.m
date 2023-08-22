
function [structOutput,structNames] = csvDataLogExtractor(name,options,varargin)

% csvDataLogExtractor
%
% HELP:
% use this function to retrieve a struct that contains as fields all the arrays of the .csv
% file you're extracting.
% 
% INPUT:
% - name:       "FILE.csv" which you want to open
% - options:    set options to "raw" if you want the output timestamps in seconds instead of
%               microseconds, default is seconds
% - varargin:   insert a cell array of names for variables if you recognize the
%               readtable returns Var1, Var2, ...
% 
% 
% OUTPUT: 
% - structOutput:   output struct with .csv information
% - structNames:    array containing the names of the fields of the outputStruct
% (non necessary, but it can be usefull for dynamic programming)

%{ 
csvDataLogExtractor()

Author: Marco Marchesi - GNC
        marco.marchesi@skywarder.eu

-       Release: 03/09/2022
%}

if nargin < 2
    options = "sec"; % default
end

structExtractTable = readtable(name);
varNames = structExtractTable.Properties.VariableNames;

readCheck = 0;
for j = 1 : length(varNames)
        if contains(varNames{j}, 'Var')
            readCheck = readCheck +1;
        end
end
if readCheck == length(varNames)
    varNames = varargin{1};
end

structOutput = struct();
for i = 1:size(structExtractTable,2)
    structOutput.(varNames{i}) = structExtractTable(:,i).Variables;
end

structNames = convertCharsToStrings(fieldnames(structOutput));



% adjust timestsamps ( timestamps restart from zero when a certain value is
% reached ) 

if options ~= "raw"
overflow = 2^32;  

    for j = 1 : length(structNames)
        if contains(structNames(j), 'imestamp')
            
            idx_save = 0;
            for i = 2:length(structOutput.(structNames(j)))    
                if structOutput.(structNames(j))(i)<structOutput.(structNames(j))(i-1) && abs(structOutput.(structNames(j))(i)-structOutput.(structNames(j))(i-1))>overflow/2
                    idx_save = [idx_save, i-1];
                end            
            end
            idx_save = [idx_save, length(structOutput.(structNames(1)))];    
            
            for k = 2:length(idx_save)
                structOutput.(structNames(j))(idx_save(k-1)+1:idx_save(k)) = structOutput.(structNames(j))(idx_save(k-1)+1:idx_save(k)) + (k-2)*overflow;
            end

            % transform in seconds
            if options == "sec"
                dt = 1e6;
                structOutput.(structNames(j)) = structOutput.(structNames(j))/dt;
            end
        end
    end
end

