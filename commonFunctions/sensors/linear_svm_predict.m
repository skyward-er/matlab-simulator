function [classification] = linear_svm_predict(Mdl, x)

%{
HELP:
classification function for sensor fault detection

INPUT: 
- Mdl =  model of the support vector machine
- x = features

OUTPUT: 
- classification = true or false, if true then it's faulty

%}
    scale = Mdl.Scale;
    beta = Mdl.Beta;
    bias = Mdl.Bias;
    %normalization of the feautures
    mu = Mdl.Mu;
    sigma = Mdl.Sigma;
    %x = bsxfun(@minus,x,mu);
    x = x - mu;
    x = x./sigma;
    %calculating the prediction score of the linear kernel
    score = -(((x/scale))*beta + bias);
    %labeling based on the sign of the score 
    if(score < 0 || isnan(score))
        classification = true; %classifies as faulty
    else
        classification = false;
    end
end