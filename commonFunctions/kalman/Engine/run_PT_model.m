function [P_measured, t] = run_PT_model(P_real, t_all)

% Sensor's characteristics
frequency = 50;
sampleTime = 1/frequency;
saturation = [0, 700];
Nbits = 16;
bias = 0;   % Or generate randomly (0 default)
resolution = (saturation(2) - saturation(1))/2^Nbits;
noiseVariance = 0.32;
bandwidth = 10;

% Timestamp
t = 0 : sampleTime : t_all(end);
P_measured = zeros(size(P_real, 1), length(t));

for ii = 1:size(P_measured, 1)
    for k = 1:size(P_measured, 2)

        idx_P = find(t_all <= t(k), 1, "last");
        P = P_real(ii, idx_P);
        P_measured(ii, k) = P + bias + noiseVariance*randn();

        if P_measured(ii, k) >= saturation(2)
            P_measured(ii, k) = saturation(2);
        elseif P_measured(ii, k) <= -saturation(2)
            P_measured(ii, k) = -saturation(2);
        end

    end
end

end