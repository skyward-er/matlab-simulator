function servoTauAcc = tauFromDrag(z,Vnorm,ap,dap,ddap,CA,settings)

% 2022 marco.marchesi@skywarder.eu
% HELP
% function to retrieve the servo motor time constant as a function of Mach
%
%


% retrieve atmospherical parameters
[T,a,P,rho] = atmosisa(z);

% geometric parameters
theta = pi/2 - atan(2*(settings.arb.guidePol(1)*settings.arb.R*sin(42-ap)+settings.arb.guidePol(2)));

ext_ddot = settings.arb.extPol(1)*(12*ap^2*dap^2+4*ap^3*ddap)+...
    settings.arb.extPol(2)*(6*ap*dap^2 + 3*ap*ddap)+...
    settings.arb.extPol(3)*(2*dap^2+ap*ddap);

S = settings.arb.surfPol*ap;


% compute friction due to aerodynamic forces
F_A = 0.5*rho*(Vnorm^2)*S*CA*settings.arb.mu;

% compute torque load on servo
C_inertia = 7/12*settings.arb.mb*settings.arb.R*ddap + (settings.arb.ma*ext_ddot)*settings.arb.R*sin(settings.servo.maxAngle-ap-theta);
C_aero =  F_A*settings.arb.R*sin(settings.servo.maxAngle-deg2rad(21)-ap-theta);
C_load = 3*(C_inertia + C_aero); %3* because the other quantities are copmuted for a single airbrake

totalMass = (settings.arb.ma + settings.arb.mb).*3;
acc = (settings.servo.maxTorque + C_load)./totalMass;
servoTauAcc = settings.servo.maxSpeed./(5*acc);