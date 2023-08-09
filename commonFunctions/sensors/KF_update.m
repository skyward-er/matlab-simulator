function [P,X] = KF_update(P,X,yy,R,H)
    Inn = yy - H*X;
    S = H*P*H' + R; %H is Ct (R is sensor covariance)
    K = P*H'/S;
    
    X = X + K*Inn;
    P = P - K*H*P;
end