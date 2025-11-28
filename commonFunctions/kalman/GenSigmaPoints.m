function [sigmapts, weights] = GenSigmaPoints(x, P, n, k)

%% Used datas
x = reshape(x, [], 1);
[dist, flag] = chol((n+k)*P);
if flag
    error("GenSigmaPoints: the matrix (n+k)P is NOT SPD")
end
%% Weights Computation
weights = zeros(1, 2*n+1);
weights(1) = k/(n+k);
weights(2:end) = 0.5/(n+k);

%% Sigma Points computation
sigmapts = zeros( n, 2*n+1);
sigmapts(:, 1) = x;
sigmapts(:, 2:end) = x + [dist, -dist];

end