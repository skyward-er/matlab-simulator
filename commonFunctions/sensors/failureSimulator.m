% Author: Angelo G. Gaillet
% Release date: 31/07/2022
%
% This class allows for simulation of common sensor failures

classdef failureSimulator %class delcaration

    properties (Access = private) %attributes private
        failureType;
        offset;
        lambda;
        sigmaDeg;
        sigmaIS;
        tError;
        gain;
        value;
        likelihoodFS;
        likelihoodIS;
        alsoNegSpikes;
        severity;
        randomSpikeAmpl;
        satMax;
        satMin;
        settings;

        frozenValue;
    end

    methods

        function obj = failureSimulator()%method modifies the class, constructor
            obj = obj.reset();
        end

        function obj = reset(obj) %method
            obj.failureType = 'None';
            obj.offset = 0;
            obj.lambda = 0;
            obj.sigmaDeg = 0;
            obj.sigmaIS = 0;
            obj.tError = 0;
            obj.gain = 1;
            obj.value = 0;
            obj.likelihoodFS = 0;
            obj.likelihoodIS = 0;
            obj.alsoNegSpikes = false;
            obj.severity = 0;
            obj.randomSpikeAmpl = false;
            obj.satMax = inf;
            obj.satMin = -inf;
            obj.settings.saturateFlag = false;
            obj.settings.bias = false;
            obj.settings.drift = false;
            obj.settings.degradation = false;
            obj.settings.freezing = false;
            obj.settings.calerr = false;
            obj.settings.fs = false;
            obj.settings.is = false;

            obj.frozenValue = nan;
        end

        function obj = setErrorTime(obj, startTimestamp)
            if startTimestamp >= 0
                obj.tError = startTimestamp;
            else
                error('Time needs to be positive')
            end
        end

        function [obj, sensorData] = applyFailure(obj, sensorData, timestamp)
            for i = 1:length(timestamp)
                if timestamp(i) >= obj.tError
                    if obj.settings.bias
                        sensorData(i) = sensorData(i) + obj.offset;
                    end
                    if obj.settings.drift
                        sensorData(i) = sensorData(i) + obj.lambda * (timestamp(i) - obj.tError);
                    end
                    if obj.settings.degradation
                        sensorData(i) = sensorData(i) - obj.sigmaDeg + 2*obj.sigmaDeg*rand;
                    end
                    if obj.settings.freezing
                        if timestamp(i) == obj.tError
                            obj.frozenValue = sensorData(i);
                        end
                        sensorData(i) = obj.frozenValue;
                    end
                    if obj.settings.calerr
                        sensorData(i) = sensorData(i)*obj.gain;
                    end
                    if obj.settings.fs
                        if rand > (1 - obj.likelihoodFS)
                            sensorData(i) = obj.value;
                        end
                    end
                    if obj.settings.is
                        if rand > (1 - obj.likelihoodIS)
                            if obj.randomSpikeAmpl == true
                                randVal = rand;
                            else
                                randVal = 1;
                            end
                            if obj.alsoNegSpikes
                                sensorData(i) = sensorData(i) -obj.sigmaIS + 2*obj.sigmaIS*randVal;
                            else
                                sensorData(i) = sensorData(i) +obj.sigmaIS*randVal;
                            end
                        end
                    end
                else
                    if obj.settings.freezing
                        obj.frozenValue = sensorData(i);
                    end
                end
            end

            if obj.settings.saturateFlag
                sensorData = obj.saturate(sensorData);
            end

        end

        function obj = setOffset(obj, offset)
            obj.offset = offset;
            obj.settings.bias = true;
        end

        function obj = setDrift(obj, lambda)
            obj.lambda = lambda;
            obj.settings.drift = true;
        end

        function obj = setDegradation(obj, sigma)
            obj.sigmaDeg = sigma;
            obj.settings.degradation = true;
        end

        function obj = setFreezing (obj)
            obj.settings.freezing = true;
        end

        function obj = setCalibrationError(obj, gain)
            obj.gain = gain;
            obj.settings.calerr = true;
        end

        function obj = setFixedSpiking(obj, value, likelihood)
            if (likelihood > 0 && likelihood  < 1)
                obj.value = value;
                obj.likelihoodFS = likelihood;
                obj.settings.fs = true;
            else
                error('The likelihood must be between 0 and 1')
            end
        end

        function obj = setIncrementalSpiking(obj, sigma, likelihood, severity, alsoNegative, random)
            if islogical(random)
                if islogical(alsoNegative)
                    if (likelihood > 0 && likelihood  < 1)
                        obj.sigmaIS = sigma;
                        obj.likelihoodIS = likelihood;
                        obj.severity = severity;
                        obj.alsoNegSpikes = alsoNegative;
                        obj.settings.is = true;
                        obj.randomSpikeAmpl = random;
                    else
                        error('The likelihood must be between 0 and 1')
                    end
                else
                    error('alsoNegative must be either False (for only positive spikes) or True (for both positive and negative spikes)');
                end
            else
                error('random must be either False (for randomness in the increments) or True (for fixed increments)');
            end
        end

        function obj = setSensorSaturation(obj, min, max)
            obj.satMax = max;
            obj.satMin = min;
            obj.settings.saturateFlag = True;
        end

    end

    methods (Access = private)

        function data = saturate(data)
            data = max(obj.satMin, min(obj.satMax, data));
        end

    end

end