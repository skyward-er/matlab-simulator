function [WES] = run_WES(vel_est,WES)

%% WES acquisition

% if WES.N < WES.N_cal
% 
%     % here there was a if t> N*period, which I think is useless because now
%     % it goes at a discrete frequency set outside of the function
%     WES.N = WES.N + 1;
% 
%     WES.A(WES.N,:) = vel_est;
%     WES.b(WES.N,1) = norm(vel_est)^2;
% 
%     % update mean
%     WES.V_mean = (WES.V_mean.*(WES.N-1)+vel_est)./WES.N;
%     WES.V2_mean = (WES.V2_mean.*(WES.N-1)+norm(vel_est)^2)./WES.N;
% 
% 
%     %% calibration 
%     if WES.N == WES.N_cal
%         WES.A = WES.A - mean(WES.A,1);
%         WES.b = 0.5*(WES.b - mean(WES.b));
%         WES.wind_est = (WES.A'*WES.A)\(WES.A'*WES.b);
% %         WES.V_h_i = norm(vel_est-WES.wind_est);
%     end
% 
% else
%     if WES.state == 1                                               % Guidance NOT running, only calibrations
        % ------ Recursive Least Squares ------ 
        % New number of samples
        WES.N = WES.N + 1;

        % Updated mean
        WES.V_mean = (WES.V_mean.*(WES.N-1)+vel_est)./WES.N;
        WES.V2_mean = (WES.V2_mean.*(WES.N-1)+norm(vel_est)^2)./WES.N;

        % RLS
        phi = vel_est-WES.V_mean;
        y = 0.5*(norm(vel_est)^2 - WES.V2_mean);
        Ep = y - phi*WES.wind_est';
        beta = WES.fFactor + phi*WES.Funv*phi';
        WES.Funv = (1/WES.fFactor)*(WES.Funv - (1/beta)*WES.Funv*(phi'*phi)*WES.Funv); % check consistency of the    ./     in     1./ beta
        K = WES.Funv*phi';
        WES.Funv = 0.5*(WES.Funv + WES.Funv');
        WES.wind_est = WES.wind_est + K'*Ep;
%         WES.V_h_i = norm(WES.V_mean-wind_est);
                      % update and save for next iteration
%     else
%         WES = WES;
     % end
        

end

