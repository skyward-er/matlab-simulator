function [P,X] = KF(P1,x1p,R,y,F,H,sensor_num)
    %Q = [0.04 0 0; 0 0.5 0; 0 0 1];
    
    Q = [1 0 0; 0 1 0; 0 0 1];
    if(sensor_num < 1 || sensor_num > 3)
        error("sensor_num is out of bound and its " + sensor_num)
    end
    [P,X] = KF_prediction(P1,x1p,Q,F);

    for i = 1:sensor_num
        
        [P, X] = KF_update(P, X, y(i) , R(i) , H);
    end
end