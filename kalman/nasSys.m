classdef nasSys
% Navigation and attitude system - This class contains the MEKF implementing the NAS
% Author: Angelo G. Gaillet
% Skyward Experimental Rocketry | ELC-SCS Dept | electronics@skywarder.eu
% email: angelo.gaillet@skywarder.eu
% Release date: 24/07/2022
    properties(Constant)
        numberOfStates = 13;
        idxPos         = 1;
        idxVel         = 4;
        idxQuat        = 7;
        idxBias        = 11;
        condthrs       = 1e-7;
    end

    properties(Dependent)
        gravityNed  
        magneticNed
        matrixQlin
        matrixQquat
        matrixFlin
        matrixFquat
        matrixGquat
    end
    
    properties

        % SET THESE CONSTANTS BEFORE USE
%         latitude0  = % [decimal degrees]
%         longitude0 = % [decimal degrees]
%         altitude0  = % [m] 
%         temperature0 = % at Ground Level [K]
%         launchDate = [] % [aaaa, mm, dd]

        temperature0 = 300.19; %Reference temperature at GL for Euroc[K]
        launchDate = [2021, 10, 15];

        latitude0 = 45.5;
        longitude0 = 9.16;
        altitude0 = 204.93;
        %

                dt = 1/50;
                x  = zeros(13,1); 
                P  = [  1     0    0    0   0   0   0    0    0    0    0    0    0;
                        0     1    0    0   0   0   0    0    0    0    0    0    0;
                        0     0    1    0   0   0   0    0    0    0    0    0    0;
                        0     0    0    1   0   0   0    0    0    0    0    0    0;
                        0     0    0    0   1   0   0    0    0    0    0    0    0;
                        0     0    0    0   0   1   0    0    0    0    0    0    0;
                        0     0    0    0   0   0   0.01 0    0    0    0    0    0;
                        0     0    0    0   0   0   0    0.01 0    0    0    0    0;
                        0     0    0    0   0   0   0    0    0.01 0    0    0    0;
                        0     0    0    0   0   0   0    0    0    0.01 0    0    0;
                        0     0    0    0   0   0   0    0    0    0    0.01 0    0;
                        0     0    0    0   0   0   0    0    0    0    0    0.01 0;
                        0     0    0    0   0   0   0    0    0    0    0    0    0.01;];
                  
            sigmaB = 1e-4;
            sigmaW = 0.3;
          sigmaPre = 0.1;
          sigmaGPSxy = 0.5;
          sigmaMag = 5;
          sigmaPos = 10;
          sigmaVel = 10;
          sigmaPit = 10;
          apogee   = false;
    end
    methods
        
        function obj = nasSys(x0,P)
                if nargin == 2
                obj.x = x0;
                obj.P = P;
                elseif nargin ==1
                obj.x = x0;
                else
                end
            end

            % Initialization
            function [x0, obj] = initializeState(obj,accelerometer,magnetomenter)
                    N = 0;   E = 0;   D = -obj.altitude0;
                   Vn = 0;  Ve = 0;  Vd = 0;
                    q = nedTriad(obj,accelerometer,magnetomenter);
                   x0 = [N; E; D; Vn; Ve; Vd; q; 0; 0; 0];
                obj.x = x0;
            end
             
            function obj = initializeVariances(obj)
                obj.matrixQlin  = get.matrixQlin(obj);
                obj.matrixQquat = get.matrixQquat(obj);
                obj.matrixFlin  = get.matrixFlin(obj);
                obj.matrixFquat = get.matrixFquat(obj);
                obj.matrixGquat = get.matrixGquat(obj);
            end
            
            function q = nedTriad(obj,accelerometer,magnetomenter)
                R1  = obj.gravityNed/norm(obj.gravityNed); %(in c++ -gravityNed bc acc measures normal acceleration)
                R2  = cross(obj.gravityNed,obj.magneticNed)/norm(cross(obj.gravityNed,obj.magneticNed));
                R3  = cross(R1,R2);
                
                r1  = accelerometer/norm(accelerometer);
                r2  = cross(accelerometer,magnetomenter)/norm(cross(accelerometer,magnetomenter));
                r3  = cross(r1,r2);
                dcm = [R1, R2, R3]* [r1, r2, r3]';
                q = dcm2quat(dcm)';
            end
            
            % Prediction
            function [x, P, a, obj]   = predict(obj, accelerometers, gyroscopes)
               pos   = obj.x(obj.idxPos:obj.idxPos+2);
               vel   = obj.x(obj.idxVel:obj.idxVel+2);
               q     = obj.x(obj.idxQuat:obj.idxQuat+3);
               bias  = obj.x(obj.idxBias:obj.idxBias+2);
               
               R     = body2ned(obj,q);
               pos   = pos + vel * obj.dt;
               a     = R*accelerometers;
               a     = a + obj.gravityNed;
               vel   = vel + (R*accelerometers + obj.gravityNed)*obj.dt;

               omega = gyroscopes - bias;
               Omega = [     0        omega(3)    -omega(2)    omega(1);
                          -omega(3)     0          omega(1)    omega(2);
                           omega(2)  -omega(1)       0         omega(3);
                          -omega(1)  -omega(2)    -omega(3)      0;    ];
               q     = q + 0.5 * Omega * obj.dt * q;
               q     = q/norm(q);
               
               obj.x = [pos; vel; q; bias];
               x     = obj.x;
               
               obj   = propagateVariance(obj);
               P     = obj.P;
            end
            
            function [obj] = propagateVariance(obj)

                Pq    = obj.P(obj.idxQuat+1:obj.idxBias+2,obj.idxQuat+1:obj.idxBias+2);
                Pq    = obj.matrixFquat*Pq*obj.matrixFquat'+obj.matrixGquat*obj.matrixQquat*obj.matrixGquat';
                
                Pl    = obj.P(obj.idxPos:obj.idxVel+2,obj.idxPos:obj.idxVel+2);
                Pl    = obj.matrixFlin*Pl*obj.matrixFlin' + obj.matrixQlin;
                
                obj.P(obj.idxQuat+1:obj.idxBias+2,obj.idxQuat+1:obj.idxBias+2) = Pq;
                obj.P(obj.idxPos:obj.idxVel+2,obj.idxPos:obj.idxVel+2)   = Pl;
            end

            function [x, P, res, obj]     = correctBaro(obj,barometer)
                H = [0,0,0,0,0,0];
                [temp, ~, ~, ~] = atmosisa(-obj.x(3));
                a = 0.0065;
                n = 9.80665/(287.05*a);
                P0   = 101468.932468160;
                H(1,3) = a*n*P0*((1 - a*obj.x(3)/temp)^(-n -1))/temp;
                R      = obj.sigmaPre;
                Pl     = obj.P(obj.idxPos:obj.idxVel+2,obj.idxPos:obj.idxVel+2);
                S      = H*Pl*H' + R;
                [~, ~, press, ~] = atmosisa(-obj.x(3));
                if cond(S) > obj.condthrs 
                    e      =   barometer - press;
                    K      =   Pl*H'/S;
                    obj.x(obj.idxPos:obj.idxVel+2) =   obj.x(obj.idxPos:obj.idxVel+2) + K*e;
                    Pl     =   (eye(6) - K*H)*Pl;
                    obj.P(obj.idxPos:obj.idxVel+2,obj.idxPos:obj.idxVel+2) =   Pl;
                end
                x      =   obj.x;
                P      =   obj.P;
                res    =   barometer - press;
            end
            
            function [x, P, res, obj]     = correctGPS(obj, GPS, fix)
                % this method corrects horizontal position and velocity, it does not correct vertically
                res = 0;
                if fix
                    % GPS contains lat, lon, height, vN, vE, vD (we could also get speed, track, num_sat)
                    
                    % convert degrees of latitude and longitude to distance in meters
                    lat = GPS(1);
                    mperdeglat = 111132.95225;
                    mperdeglon = abs(111412.87733 * cosd(lat));

                    gps(1) = mperdeglat .* (GPS(1) - obj.latitude0);
                    gps(2) = mperdeglon .* (GPS(2) - obj.longitude0);
                    gps(3:4) = [GPS(4), GPS(5)];

                    % correction
                    H      = [1 0 0 0 0 0;
                              0 1 0 0 0 0;
                              0 0 0 1 0 0;
                              0 0 0 0 1 0];
                    R      = obj.sigmaGPSxy * eye(4);
                    Pl     = obj.P(obj.idxPos:obj.idxVel+2,obj.idxPos:obj.idxVel+2);
                    S      = H*Pl*H' + R;
                    if cond(S) > obj.condthrs 
                        e      =   gps' - H*obj.x(obj.idxPos:obj.idxVel+2);
                        K      =   Pl*H'/S;
                        obj.x(obj.idxPos:obj.idxVel+2) =   obj.x(obj.idxPos:obj.idxVel+2) + K*e;
                        Pl     =   (eye(6) - K*H)*Pl;
                        obj.P(obj.idxPos:obj.idxVel+2,obj.idxPos:obj.idxVel+2) =   Pl;
                        res    =   gps' - H*obj.x(obj.idxPos:obj.idxVel+2);
                    end
                end
                x      =   obj.x;
                P      =   obj.P;
            end
                        
            function [x, P, obj]          = correctMag(obj, mag)
                if norm(mag) < 3*norm(obj.magneticNed)
                    q     = obj.x(obj.idxQuat:obj.idxQuat+3);
                    A     = body2ned(obj,q); 
                    R     = obj.sigmaMag*eye(3);
                    mag   = mag/norm(mag);

                    h = A * mag; %magnetometers in ned frame
                    r_mag = [sqrt(h(1)^2 + h(2)^2); 0; h(3)];
                    mEst = A' * r_mag;


                    M     = [  0      -mEst(3)   mEst(2);
                             mEst(3)    0       -mEst(1);
                            -mEst(2)     mEst(1)   0;]; 
                    
                    H     = [M zeros(3,3)];
                    Pq    = obj.P(obj.idxQuat+1:obj.idxBias+2,obj.idxQuat+1:obj.idxBias+2);
                    S     = H*Pq*H'+R;
                    if cond(S) > obj.condthrs
                    K     = Pq*H'/S;
                    e     = mag - mEst;       
                    dx    = K*e;
                    r     = [0.5*dx(1:3); sqrt(1-0.25*dx(1:3)'*dx(1:3))];

                    obj.x(obj.idxQuat:obj.idxQuat+3)     = quatProd(obj,r,obj.x(obj.idxQuat:obj.idxQuat+3));     
                    obj.x(obj.idxBias:obj.idxBias+2)    = obj.x(obj.idxBias:obj.idxBias+2) + dx(4:6);
                    Pq              = (eye(6) - K*H)*Pq*(eye(6) - K*H)'+ K*R*K';  
                    obj.P(obj.idxQuat+1:obj.idxBias+2,obj.idxQuat+1:obj.idxBias+2)= Pq;
                    end
                end
                    P               = obj.P;
                    x               = obj.x;
            end

            function [x, P, obj]          = correctAcc(obj, acc)
                if (obj.apogee == true)
                    % Use only after apogee (or after T_burn_in)
                    if norm(acc) < 1.45*norm(obj.gravityNed)
                        q     = obj.x(obj.idxQuat:obj.idxQuat+3);
                        A     = body2ned(obj,q)';
                        R     = obj.sigmaMag*eye(3);
                        acc   = acc/norm(acc);
                        aEst  = A*obj.gravityNed;
                        M     = [  0      -aEst(3)   aEst(2);
                                 aEst(3)    0       -aEst(1);
                                -aEst(2)     aEst(1)   0;]; 
                        H     = [M zeros(3,3)];
                        Pq    = obj.P(obj.idxQuat+1:obj.idxBias+2,obj.idxQuat+1:obj.idxBias+2);
                        S     = H*Pq*H'+R;
    
                        K     = Pq*H'/S;
                        e     = acc - aEst;       
                        dx    = K*e;
                        r     = [0.5*dx(1:3); sqrt(1-0.25*dx(1:3)'*dx(1:3))];
    
                        obj.x(obj.idxQuat:obj.idxQuat+3)     = quatProd(obj,r,obj.x(obj.idxQuat:obj.idxQuat+3));     
                        obj.x(obj.idxBias:obj.idxBias+2)    = obj.x(obj.idxBias:obj.idxBias+2) + dx(4:6);
                        Pq              = (eye(6) - K*H)*Pq*(eye(6) - K*H)'+ K*R*K';  
                        obj.P(obj.idxQuat+1:obj.idxBias+2,obj.idxQuat+1:obj.idxBias+2)= Pq;
                     end
                end
                    P               = obj.P;
                    x               = obj.x;
            end

            function [x, P, res, obj]     = correctPitot(obj,totalP, staticP)
                %totalP: total pressure measured by the pitot tube
                %staticP: static Pressure
                p = staticP;
                p0 = totalP;
                rho = 1.225; %air density
                c = 343; %speed of sound
                gamma = 1.4; %heat capacity ratio of dry air

                if totalP >= staticP && totalP ~= 0 && obj.apogee == false
                    if norm(obj.x(obj.idxVel : obj.idxVel+2))/c >= 0.9419
                        vPitot = -sqrt(abs(-2*c^2 + sqrt((4*c^4 + 8*c^2*(p0-p)/rho))));
                    else
                        vPitot = -sqrt(c^2*2/(gamma - 1)*(abs((p0./p).^((gamma -1 )/gamma) - 1)));
                    end

                    H = zeros(1, obj.idxVel+2);
                    H(1,6) = 1;

                    R      = obj.sigmaPit;
                    Pl     = obj.P(obj.idxPos:obj.idxVel+2,obj.idxPos:obj.idxVel+2);
                    S      = H*Pl*H' + R;

                    if cond(S) > obj.condthrs
                        e      =   vPitot - obj.x(6);
                        K      =   Pl*H'/S;
                        obj.x(obj.idxPos:obj.idxVel+2) =   obj.x(obj.idxPos:obj.idxVel+2) + K*e;
                        Pl     =   (eye(obj.idxVel+2) - K*H)*Pl;
                        obj.P(obj.idxPos:obj.idxVel+2,obj.idxPos:obj.idxVel+2) =   Pl;
                    end
                    res    =   obj.x(6) - vPitot;
                else
                    res = NaN;
                end
                x      =   obj.x;
                P      =   obj.P;
            end
            
            function Qq = get.matrixQquat(obj)
                Qq      =   [(obj.sigmaW^2*obj.dt+(1/3)*obj.sigmaB^2*obj.dt^3)*eye(3)  0.5*obj.sigmaB^2*obj.dt^2*eye(3);
                                      0.5*obj.sigmaB^2*obj.dt^2*eye(3)                     obj.sigmaB^2*obj.dt*eye(3)];

            end
            
            function Ql = get.matrixQlin(obj)
                Ql      =   [obj.sigmaPos*eye(3)     zeros(3,3);
                                 zeros(3,3)    obj.sigmaVel*eye(3)];
            end
            
            function Fl = get.matrixFlin(obj)
                
                      Fl  =   [1    0    0    obj.dt  0       0;
                               0    1    0    0       obj.dt  0;
                               0    0    1    0       0       obj.dt;
                               0    0    0    1       0       0;
                               0    0    0    0       1       0;
                               0    0    0    0       0       1;];
            end
            
            function Fq = get.matrixFquat(obj)
                Fq    = [ -eye(3)       -eye(3)*obj.dt;
                           zeros(3,3)   eye(3)];
            end
               
            function Gq = get.matrixGquat(obj)
                Gq    = [ -eye(3)       zeros(3,3);
                           zeros(3,3)   eye(3)];
            end
            
            
            
            function g = get.gravityNed(obj)
                g = gravitywgs84(obj.altitude0, obj.latitude0)*[0; 0; 1]; 
            end
            
            function m = get.magneticNed(obj)
                date = decyear(obj.launchDate);
                m = wrldmagm(obj.altitude0, obj.latitude0, obj.longitude0, date,'2020'); 
            end
            
            % Utils
            function R = body2ned(~,q)
                R       = [q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2,       2*(q(1)*q(2) - q(3)*q(4)),              2*(q(1)*q(3) + q(2)*q(4));
                           2*(q(1)*q(2) + q(3)*q(4)),               -q(1)^2 + q(2)^2 - q(3)^2 + q(4)^2,     2*(q(2)*q(3) - q(1)*q(4));
                           2*(q(1)*q(3) - q(2)*q(4)),               2*(q(2)*q(3) + q(1)*q(4)),              -q(1)^2 - q(2)^2 + q(3)^2 + q(4)^2];
            end

            function quat = quatProd(~, quat1, quat2 )
                qv1 = quat1(1:3);
                qs1 = quat1(4);
                qv2 = quat2(1:3);
                qs2 = quat2(4);

                quat = [qs1 * qv2 + qs2 * qv1 - cross( qv1, qv2 ) ;
                        qs1 * qs2 - dot( qv1, qv2 )        ];
                quat = quat / norm(quat);
            end
    end
end
