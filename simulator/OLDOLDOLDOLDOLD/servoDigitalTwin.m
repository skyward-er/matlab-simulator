% digital twin

% HELP
% this script aims to give a full representation of the servo motor and air-braking
% system in order to access a more profund understanding of the
% behaviour of the system.
% Here it is proposed the most accurate model we could produce, so that
% when it comes to control design, the controller can be tested on a very
% precise model although it is designed on a simplified version of the
% system, and its reliability can be checked.
%
% thru symbolic toolbox it's possible to check the trends without actually
% knowing the values:
%
%
%
% some tips on symbolic toolbox:
%
% if you need to construct functions through symb toolbox and then evaluate
% those functions, consider transforming the symbolic version of that
% function to a matlab anonymous function through the command
% -matlabFunction(fun)-, otherwise you should use -subs(fun,old,new)- where
% "old" is the variable in the "fun" function you want to replace with the
% "new" one.
%
% integration: just use int(fun)
% differentiation: just use diff(fun,variable) variable to specify w.r.t.
% which variable you want to differenciate.
%


%% data
clear; close all; clc;

filePath = fileparts(mfilename('fullpath'));
currentPath = pwd;
if not(strcmp(filePath, currentPath))
    cd (filePath);
    currentPath = filePath;
end

addpath(genpath(currentPath));

configSimulator;
configControl;
configReferences;
%%% system data:

%interpolation coefficients for guide
guide.p(1) = -9.4265; 
guide.p(2) = 0.5337;

% interpolation coefficients for airbrake surface
Surf.p = 0.009564; % alpha

% structure data
struttura.S_max = Surf.p * settings.servo.maxAngle;              % [m^2] max aerobrake surface
struttura.m_a = 150e-3;                    % [kg] mass of 1 arb + 1 guide
struttura.m_b = 20e-3;                     % [kg] mass of 1 rod of the tristar
struttura.mu = 0.05;                       % friction coef. of bought linear guides (pessimistic,should be 0.005)
struttura.mu_g = 0.1;                       % friction coef. between cam follower and arb
struttura.alpha_max =  47 * (pi/180);      % [rad] alpha for retracted - BEFORE THE CHANGE OF REFERENCE TO 0-68
struttura.alpha_min = -21 * (pi/180);      % [rad] alpha for extracted - BEFORE THE CHANGE OF REFERENCE TO 0-68
struttura.R = 66e-3;                       % [m] length of the rod which moves the arb (OT)

%%% air parameters

[~,c_0,~,rho_0] = atmosisa(0); 
[~,c_3000,~,rho_3000] = atmosisa(3000);

% we are interested in a flight regime that never exceeds Mach 0.8,
% so this is set to be the maximum value for speed in graphs.

V_max = c_0*0.85; 

% % [~,~,~,rho] = atmosisa(z);




%
%
%
%
%
%
%
%
%
%% part 1 - geometry parameters, static forces
%
%
%
%
%
%
%
%

% in this section only the geometrical parameters are inferred, and some
% testing is done on interpolation and linearization




%% functions

% in this section all the parameters that play a role in the air braking
% system are studied, and all kinds of possible graphs are shown in order to
% give a better understanding of the system



% here are defined the variables and functions used in symbolic toolbox
syms alpha V z M(V) theta(alpha) S(alpha) Cd(V,alpha) F_A(alpha,V)  C_load

assume(alpha,'positive');
assumeAlso(alpha<=settings.servo.maxAngle);
assumeAlso(alpha,'real');

assume(V,'positive');
assumeAlso(V<=V_max);
assumeAlso(V,'real');
%% guide angle w.r.t. alpha
theta = pi/2 - atan(2*(guide.p(1)*struttura.R*sin(struttura.alpha_max-alpha)+guide.p(2)));

theta_fig = figure;
fplot(theta,[-pi pi],'r--')
hold on; grid on;
fplot(theta,[0 settings.servo.maxAngle],'k',LineWidth=2)

title('Guide angle $\theta$ w.r.t. $\alpha$',Interpreter='latex')
legend('True','Constrained interval',Interpreter='latex')
xlabel('$\alpha[rad]$',Interpreter='latex')
ylabel('$\theta[rad]$',Interpreter='latex')

% linear and quadratic interpolation 
theta_vect = matlabFunction(theta);
theta_p_interp1 = polyfit(linspace(0,settings.servo.maxAngle,69),theta_vect(linspace(0,settings.servo.maxAngle,69)),1);

% defining interpolation polynomials, theta_interp1(alpha) theta_interp2(alpha)
theta_interp1 = alpha*theta_p_interp1(1) + theta_p_interp1(2);

theta_interp1_fig = figure ;
subplot(1,2,1)
fplot(theta,[0 settings.servo.maxAngle],'r--',LineWidth=2)
hold on;grid on;
fplot(theta_interp1,[0 settings.servo.maxAngle],'k',LineWidth=2)

title('Interpolation functions for $\theta$',Interpreter='latex')
legend('True','Linear interp',Interpreter='latex')
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('$\theta$[rad]',Interpreter='latex')

% error between interpolation and true curve

error_perc_theta_1 = abs(theta-theta_interp1)/theta*100;

subplot(1,2,2)
fplot(error_perc_theta_1,[0 settings.servo.maxAngle],'k',LineWidth=2)
grid on;
title('Interpolation percentual error',Interpreter='latex')
legend('Linear',Interpreter='latex')
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('Error \%',Interpreter='latex')
axis([0 settings.servo.maxAngle 0 100])

%% linear theta with same extrema

% it is of interest to produce a linear function with the same extrema as
% the real function, because it produces better results when it comes to
% dynamic behaviour:

theta_p_linear(1) = (subs(theta,alpha,0)-subs(theta,alpha,settings.servo.maxAngle))/(-settings.servo.maxAngle);
theta_p_linear(2) = subs(theta,alpha,0);

theta_lin_ex = alpha*theta_p_linear(1)+theta_p_linear(2);

theta_linear_fig = figure; 
subplot(1,2,1)
fplot(theta,[0 settings.servo.maxAngle],'r--',LineWidth=2)
hold on;grid on;
fplot(theta_lin_ex,[0 settings.servo.maxAngle],'k',LineWidth=2)

title('Interpolation functions for $\theta$',Interpreter='latex')
legend('True','Linear interp',Interpreter='latex')
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('$\theta$[rad]',Interpreter='latex')

% error between interpolation and true curve

error_perc_theta_lin = abs(theta-theta_lin_ex)/theta*100;

subplot(1,2,2)
fplot(error_perc_theta_lin,[0 settings.servo.maxAngle],'k',LineWidth=2)
grid on;
title('Interpolation percentual error',Interpreter='latex')
legend('Linear',Interpreter='latex')
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('Error \%',Interpreter='latex')
axis([0 settings.servo.maxAngle 0 100])

%% air brake center of mass position along the movement axis w.r.t. alpha 

x_g = settings.arb.extPol(1)*alpha^4+settings.arb.extPol(2)*alpha^3+settings.arb.extPol(3)*alpha^2+settings.arb.extPol(4)*alpha;

x_g_fig = figure;
fplot(x_g,[-5,5],'r--')
hold on; grid on;
fplot(x_g,[0 settings.servo.maxAngle],'k',LineWidth=2)
title('Position of CG of the airbrake w.r.t. $\alpha$',Interpreter='latex')
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('$x_g$[m]',Interpreter='latex')
legend('Function on I=[-5;5]','Constrained values',Interpreter='latex')

x_g_vect = matlabFunction(x_g);
ext.p_interp = polyfit(linspace(0, settings.servo.maxAngle, 69),x_g_vect(linspace(0,settings.servo.maxAngle,69)),1);
% syms x_g_interp(alpha)
x_g_interp = alpha*ext.p_interp(1) + ext.p_interp(2);


x_g_interp1_fig = figure;
subplot(1,2,1)
fplot(x_g,[0 settings.servo.maxAngle],'r--')
hold on; grid on;
fplot(x_g_interp,[0 settings.servo.maxAngle],'k',LineWidth=2)
title('Comparison',Interpreter='latex')
legend('True','Linear interp',Interpreter='latex')
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('$x_g$[m]',Interpreter='latex')

% error between interpolation and true curve
error_x_g = abs(x_g-x_g_interp)/abs(x_g)*100;

subplot(1,2,2)
fplot(error_x_g,[0 settings.servo.maxAngle],'k',LineWidth=1.5)
grid on;
title('Percentual error',Interpreter='latex')
axis([0 settings.servo.maxAngle 0 100])
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('$Error \%$',Interpreter='latex')

%% linear x_g with same extrema

% it is of interest to produce a linear function with the same extrema as
% the real function, because it produces better results when it comes to
% dynamic behaviour:

ext.p_lin(1)= double((subs(x_g,alpha,0)-subs(x_g,alpha,settings.servo.maxAngle))/(-settings.servo.maxAngle));
ext.p_lin(2)= double(subs(x_g,alpha,0));

x_g_lin_ex = alpha*ext.p_lin(1)+ext.p_lin(2);

x_g_linear_fig = figure;
subplot(1,2,1)
fplot(x_g,[0 settings.servo.maxAngle],'r--')
hold on; grid on;
fplot(x_g_lin_ex,[0 settings.servo.maxAngle],'k',LineWidth=2)
title('Linear $x_g$ with same extrema',Interpreter='latex')
legend('True','Linear',Interpreter='latex')
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('$x_g$[m]',Interpreter='latex')

% error between interpolation and true curve
error_perc_x_g_lin = abs(x_g-x_g_lin_ex)/(abs(x_g))*100;
subplot(1,2,2)
fplot(error_perc_x_g_lin,[0 settings.servo.maxAngle],'k',LineWidth=2)
grid on;
axis([0 settings.servo.maxAngle 0 100])
title('Percentual error',Interpreter='latex')
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('$Error \%$',Interpreter='latex')

%% done this, probably useless - may delete
% %% transposing the curve and searching the optimal - (x_g)
% 
% 
% N = 100000;
% DEN = N*10;
% 
% % read this help to understand what follows:
% %
% % N_int = 1000;
% % alpha_vec = linspace(0,settings.servo.maxAngle,N_int);
% % for i = 1:N
% % error_x_g_0 = @(alphax) abs((x.p(1).*alphax.^4+x.p(2).*alphax.^3+x.p(3).*alphax.^2+x.p(4).*alphax)-(alphax.*(x.p_interp(1)-x.p_interp(1)*(i/DEN)))/(x.p(1).*alphax.^4+x.p(2).*alphax.^3+x.p(3).*alphax.^2+x.p(4).*alphax)*100+100);
% % e_int(i) = simpcomp(0,settings.servo.maxAngle,N_int,error_x_g_0);
% % e_abs(i) = max(error_x_g_0(alpha_vec));
% % i
% % end
% % [best_error_int,index_error_int]=min(e_int);
% % [best_error_abs,index_error_abs]=min(e_abs);
% % syms alpha
% % from here the values for the indeces are derived:
% 
% index_error_int = 33038;
% index_error_abs = 32984;
% 
% x_coeff_min_err_abs = (x.p_interp(1)-x.p_interp(1)*(index_error_abs/DEN));
% x_coeff_min_err_int = (x.p_interp(1)-x.p_interp(1)*(index_error_int/DEN));
% 
% 
% x_g_interp_to0_abs =  alpha*x_coeff_min_err_abs;
% x_g_interp_to0_int =  alpha*x_coeff_min_err_int;
% 
% error_x_g_0_abs = abs(x_g-x_g_interp_to0_abs)/abs(x_g)*100;
% error_x_g_0_int = abs(x_g-x_g_interp_to0_int)/abs(x_g)*100;
% 
% 
% figure
% subplot(1,2,1)
% fplot(x_g,[0 settings.servo.maxAngle],'r--')
% hold on; grid on;
% fplot(x_g_interp_to0_abs,[0 settings.servo.maxAngle],'k',LineWidth=2)
% fplot(x_g_interp_to0_int,[0 settings.servo.maxAngle],'m',LineWidth=1.5)
% title('Comparison ($x_g$)',Interpreter='latex')
% legend('True','Linear interp $min(e)$','Linear interp $min(\int e)$',Interpreter='latex')
% xlabel('$\alpha$[rad]',Interpreter='latex')
% ylabel('$x_g$[m]',Interpreter='latex')
% 
% % error between constrained to 0 interpolation and true curve
% 
% subplot(1,2,2)
% fplot(error_x_g_0_abs,[0 settings.servo.maxAngle],'k',LineWidth=2)
% grid on; hold on;
% fplot(error_x_g_0_int,[0 settings.servo.maxAngle],'m',LineWidth=1.5)
% legend('Optimized $max(|e|)$','Optimized $\int (e)$',Interpreter='latex')
% title('Percentual error',Interpreter='latex')
% axis([0 settings.servo.maxAngle 0 100])
% xlabel('$\alpha$[rad]')
% ylabel('$Error \%$',Interpreter='latex')
% 
% % ok now, like this there can be a problem, because when alpha is 68° (max
% % value for alpha) x_g_interp and x_g (real) are different. for what I know
% % now it can be a problem but also not, need to investigate.

%% air brake exposed surface w.r.t. alpha
S = Surf.p*alpha; 

S_fig = figure;
fplot(S,[0 settings.servo.maxAngle],'k', LineWidth=2)
grid on;
title('Exposed surface of one airbrake w.r.t. $\alpha$',Interpreter='latex')
xlabel('$\alpha [rad]$',Interpreter='latex')
ylabel('$S [m^2]$',Interpreter='latex')

%% CD w.r.t. alpha and V
% Mach as a function of speed
M = V/c_0;

% for Cd (drag coefficient) is used an interpolation of parameters given by the aerodynamic
% department ( not sure it is the best representation of the system,
% perhaps aero.mat is better, BUT that is not a function )
load('CAinterpCoeffs.mat')

%%% true cd
Cd   =  coeffs.n100*M + coeffs.n200*(M^2) + ...
        coeffs.n300*(M^3) + coeffs.n400*(M^4) + ...
        coeffs.n500*(M^5) + coeffs.n600*(M^6) + ...
        coeffs.n010*x_g + coeffs.n020*(x_g^2) + ...
        coeffs.n110*x_g*M + coeffs.n120*(x_g^2)*M + ...
        coeffs.n210*x_g*(M^2) + coeffs.n220*(x_g^2)*(M^2) + ...
        coeffs.n310*x_g*(M^3) + coeffs.n320*(x_g^2)*(M^3) + ...
        coeffs.n410*x_g*(M^4) + coeffs.n420*(x_g^2)*(M^4) + ...
        coeffs.n510*x_g*(M^5) + coeffs.n520*(x_g^2)*(M^5) +coeffs.n000; %+ coeffs.n001*h;

Cd_fig = figure;
subplot(1,2,1)
fsurf(Cd,[0 V_max 0 settings.servo.maxAngle]) %fixed
grid on; hold on;
title('Drag coefficient due to aerodynamic forces of one airbrake w.r.t. $\alpha$',Interpreter='latex')
xlabel('$V [m/s]$',Interpreter='latex')
ylabel('$\alpha [rad]$',Interpreter='latex')
zlabel('$Cd [-]$',Interpreter='latex')
legend('True $x_g$',Interpreter='latex')
colorbar

%%% compute max cd
alpha_vec = linspace(0,settings.servo.maxAngle,100)';
V_vec = linspace(0,V_max,100)';

Cd_fun= matlabFunction(Cd);
for i = 1:length(alpha_vec)
    for j = 1:length(V_vec) 
Cd_vec(j,i) = Cd_fun(V_vec(j),alpha_vec(i));
    end
end
Cd_max = max(max(Cd_vec));
%%%


Cd_lin_xg   =  coeffs.n100*M + coeffs.n200*(M^2) + ...
        coeffs.n300*(M^3) + coeffs.n400*(M^4) + ...
        coeffs.n500*(M^5) + coeffs.n600*(M^6) + ...
        coeffs.n010*x_g_lin_ex + coeffs.n020*(x_g_lin_ex^2) + ...
        coeffs.n110*x_g_lin_ex*M + coeffs.n120*(x_g_lin_ex^2)*M + ...
        coeffs.n210*x_g_lin_ex*(M^2) + coeffs.n220*(x_g_lin_ex^2)*(M^2) + ...
        coeffs.n310*x_g_lin_ex*(M^3) + coeffs.n320*(x_g_lin_ex^2)*(M^3) + ...
        coeffs.n410*x_g_lin_ex*(M^4) + coeffs.n420*(x_g_lin_ex^2)*(M^4) + ...
        coeffs.n510*x_g_lin_ex*(M^5) + coeffs.n520*(x_g_lin_ex^2)*(M^5) +coeffs.n000; %+ coeffs.n001*h;

subplot(1,2,2)
fsurf(Cd_lin_xg,[0 V_max 0 settings.servo.maxAngle]) %fixed
grid on;
title('Drag coefficient due to aerodynamic forces of one airbrake w.r.t. $\alpha$',Interpreter='latex')
xlabel('$V [m/s]$',Interpreter='latex')
ylabel('$\alpha [rad]$',Interpreter='latex')
zlabel('$Cd [-]$',Interpreter='latex')
legend('Linear $x_g$',Interpreter='latex')
colorbar

%%% error
error_Cd = abs(Cd-Cd_lin_xg)/abs(Cd)*100;

cd_error_fig = figure;
fsurf(error_Cd,[0 V_max 0 settings.servo.maxAngle],LineWidth=1.5)
grid on;
title('Percentual error',Interpreter='latex')
axis([0 V_max 0 settings.servo.maxAngle 0 100])
xlabel('$\alpha$[rad]',Interpreter='latex')
ylabel('$Error \%$',Interpreter='latex')
colorbar
%% friction force between rod and guide for different values of Mach w.r.t.

F_A = 0.5*rho_0*(V^2)*S*Cd*struttura.mu;
% I don't think this is accurate, because it should depend on the 
% contact surface between airbrake and guide, but here it does not appear 
% this surface. I guess taking mu = 0.05 (which is a conservative value as 
% mu should be around 0.005) they fixed took this into account.

figure
fsurf(F_A,[0 V_max 0 settings.servo.maxAngle])
grid on;
title('Friction due to aerodynamic forces of one airbrake w.r.t. $\alpha$',Interpreter='latex')
xlabel('$\alpha [rad]$',Interpreter='latex')
ylabel('$V [m/s]$',Interpreter='latex')
zlabel('$F_A [N]$',Interpreter='latex')

%% Torque load

% this term cannot be studied without a temporal dependancy for alpha,
% because it needs the second derivative w.r.t. time for alpha and x_g. so
% it is postponed to the second part of this code.

%% a further approximation (legit) 
% studying the system with all the linearized terms the only thing that
% remains non-linear is in the Torque, and it is:
%
% sin(settings.servo.maxAngle - alpha - theta).
%
% if theta is taken linear, this expression reduces to:
%
% sin(-0.29*alpha-0.18)
%
% Now, given that alpha is an angle between 0 and 1.18 [rad], seems legit
% to study the behaviour of this sinusoidal, as we can expect it to be some
% kind of linear too.


sin_true = sin(-0.29*alpha-0.18);

sin_true_vec = matlabFunction(sin_true);
%true_sin_vec(0) = -0.179029573425824
sin_interp_coeffs1 = polyfit(deg2rad(0:68),sin_true_vec(deg2rad(0:68)),1);
sin_interp1 = sin_interp_coeffs1(1)*alpha + sin_interp_coeffs1(2);

sin_interp_coeffs2 = [(subs(sin_true,alpha,0)-subs(sin_true,alpha,settings.servo.maxAngle)) , sin_true_vec(0)];
sin_interp2 = alpha*sin_interp_coeffs2(1)/(-settings.servo.maxAngle) + sin_interp_coeffs2(2);

sin_fig = figure;
subplot(1,2,1)
fplot(sin_true,[0 settings.servo.maxAngle],'r--',LineWidth=1.5)
hold on; grid on;
fplot(sin_interp1,[0 settings.servo.maxAngle],'k',LineWidth=1.5)
fplot(sin_interp2,[0 settings.servo.maxAngle],'m',LineWidth=1)

title('Comparison on non linear term',Interpreter='latex')
legend('True','Linear','linear but moved',Interpreter='latex')
xlabel('$\alpha [rad]$',Interpreter='latex')
ylabel('$sin(\alpha_{max}-\alpha-\theta) [rad]$',Interpreter='latex')

% obs: the black curve is best fitting, but the green one has the extrema
% with the same values of the sinusoidal function, so in my opinion it is a
% better choice.
error_perc_sin_lin_1 = abs(sin_true-sin_interp1)/abs(sin_true)*100;
error_perc_sin_lin_2 = abs(sin_true-sin_interp2)/abs(sin_true)*100;

subplot(1,2,2)
fplot(error_perc_sin_lin_1,[0 settings.servo.maxAngle],'k',LineWidth=1.5)
hold on; grid on;
fplot(error_perc_sin_lin_2,[0 settings.servo.maxAngle],'m',LineWidth=1)

title('Percentual error',Interpreter='latex')
legend('Linear','Linear but moved',Interpreter='latex')
xlabel('$\alpha [rad]$',Interpreter='latex')
ylabel('$sin(\alpha_{max}-\alpha-\theta) [rad]$',Interpreter='latex')
axis([0 settings.servo.maxAngle 0 100])

%
%
%
%
%
%
%
%
%
%
%% part 2 - dynamic forces
%
%
%
%
%
%
%
%
%


%% functions

% in this section all the parameters that play a role in the air braking
% system are studied with a DYNAMIC pov. Here the aim is to get all the inertia forces and torques 
% values to study if they are negligible or not in the controller design
% phase.

% here are defined the variables and functions used in symbolic toolbox
syms t alpha(t) V z M(V) theta(alpha) S(alpha) Cd(V,alpha) F_A(alpha,V)  C_load

assume(t,'positive')

%% alpha profile
T_zero = 0.1; % zero time constant [s]
T_dead = 0.00; % dead time [s]
alpha_ref = settings.servo.maxAngle; 

% first order plus dead time alpha profile
alpha = (1-exp(-(t-T_dead)/T_zero))*alpha_ref; 

alpha_dot = diff(alpha,t);
alpha_ddot = diff(alpha_dot,t);

T_transient = 5*T_zero + T_dead;
T_interval = T_transient*5;

% plot alpha w.r.t. time
figure
subplot(1,3,1)
fplot(alpha,[0 T_interval],'k',LineWidth=1.5)
grid on;
title('$\alpha$ w.r.t. time',Interpreter='latex')
legend('$\alpha$',Interpreter = 'latex')
xlabel('t',Interpreter='latex')
ylabel('$\alpha(t)$',Interpreter='latex')

subplot(1,3,2)
fplot(alpha_dot,[0 T_interval],'k',LineWidth=1.5)
grid on;
title('$\dot\alpha$ w.r.t. time',Interpreter='latex')
legend('$\dot\alpha$',Interpreter = 'latex')
xlabel('t',Interpreter='latex')
ylabel('$\dot\alpha(t)$',Interpreter='latex')


subplot(1,3,3)
fplot(alpha_ddot,[0 T_interval],'k',LineWidth=1.5)
grid on;
title('$\ddot\alpha$ w.r.t. time',Interpreter='latex')
legend('$\ddot\alpha$',Interpreter = 'latex')
xlabel('t',Interpreter='latex')
ylabel('$\ddot\alpha(t)$',Interpreter='latex')

%% guide angle w.r.t. alpha
theta = pi/2 - atan(2*(guide.p(1)*struttura.R*sin(struttura.alpha_max-alpha)+guide.p(2)));

% defining interpolation polynomials, theta_interp1(alpha)
% theta_interp2(alpha) 
% values from the static interpolation
theta_lin = alpha*theta_p_linear(1) + theta_p_linear(2);

figure 
% subplot(1,2,1)
fplot(theta,[0 T_interval],'r--',LineWidth=2)
hold on;grid on;
fplot(theta_lin,[0 T_interval],'k',LineWidth=2)

title('Interpolation functions for $\theta$',Interpreter='latex')
legend('True','Linear interp',Interpreter='latex')
xlabel('$t [s]$',Interpreter='latex')
ylabel('$\theta$[rad]',Interpreter='latex')


%% air brake center of mass position along the movement axis w.r.t. alpha 

x_g = settings.arb.extPol(1)*alpha^4+settings.arb.extPol(2)*alpha^3+settings.arb.extPol(3)*alpha^2+settings.arb.extPol(4)*alpha;
x_g_dot = diff(x_g,t);
x_g_ddot = diff(x_g_dot,t);

x_g_lin_def =  alpha*ext.p_lin(1) + ext.p_lin(2);
x_g_lin_def_dot = diff(x_g_lin_def,t);
x_g_lin_def_ddot = diff(x_g_lin_def_dot,t);

figure
subplot(1,3,1)
fplot(x_g,[0 T_interval],'r--')
hold on; grid on;
fplot(x_g_lin_def,[0 T_interval],'k',LineWidth=2)
title('$x_g$ w.r.t. time',Interpreter='latex')
legend('True','Linear interp',Interpreter='latex')
xlabel('$t [s]$',Interpreter='latex')
ylabel('$x_g [m]$',Interpreter='latex')

subplot(1,3,2)
fplot(x_g_dot,[0 T_interval],'r--')
hold on; grid on;
fplot(x_g_lin_def_dot,[0 T_interval],'k',LineWidth=2)
title('$\dot x_g$ w.r.t. time',Interpreter='latex')
legend('True','Linear interp',Interpreter='latex')
xlabel('$t [s]$',Interpreter='latex')
ylabel('$\dot x_g [m/s]$',Interpreter='latex')

subplot(1,3,3)
fplot(x_g_ddot,[0 T_interval],'r--')
hold on; grid on;
fplot(x_g_lin_def_ddot,[0 T_interval],'k',LineWidth=2)
title('$\ddot x_g$ w.r.t. time',Interpreter='latex')
legend('True','Linear interp',Interpreter='latex')
xlabel('$t [s]$',Interpreter='latex')
ylabel('$\ddot x_g [m/s^2]$',Interpreter='latex')



%% air brake exposed surface w.r.t. alpha
S = Surf.p*alpha; 

figure
fplot(S,[0 T_interval],'k', LineWidth=2)
grid on;
title('Exposed surface of one airbrake w.r.t. time',Interpreter='latex')
xlabel('$t [s]$',Interpreter='latex')
ylabel('$S [m^2]$',Interpreter='latex')




%% Torque load
% Here the torque can be studied, as now with an alpha profile as a
% function of time it is possible to infer on inertia.
% note that the friction force, as it does not depend on the derivatives of
% alpha, is computed here but not plotted.

%Cd
% Mach as a function of speed
M = V/c_0;

Cd   =  coeffs.n100*M + coeffs.n200*(M^2) + ...
        coeffs.n300*(M^3) + coeffs.n400*(M^4) + ...
        coeffs.n500*(M^5) + coeffs.n600*(M^6) + ...
        coeffs.n010*x_g + coeffs.n020*(x_g^2) + ...
        coeffs.n110*x_g*M + coeffs.n120*(x_g^2)*M + ...
        coeffs.n210*x_g*(M^2) + coeffs.n220*(x_g^2)*(M^2) + ...
        coeffs.n310*x_g*(M^3) + coeffs.n320*(x_g^2)*(M^3) + ...
        coeffs.n410*x_g*(M^4) + coeffs.n420*(x_g^2)*(M^4) + ...
        coeffs.n510*x_g*(M^5) + coeffs.n520*(x_g^2)*(M^5) +coeffs.n000; %+ coeffs.n001*h;

%friction force
F_A = 0.5*rho_0*(V^2)*S*Cd*struttura.mu; 
% I don't think this is accurate, because it should depend on the 
% contact surface between airbrake and guide, but here it does not appear 
% this surface. I guess taking mu = 0.05 (which is a conservative value as 
% mu should be around 0.005) they took this into account.

%torques
C_inertia = 7/12*struttura.m_b*struttura.R*alpha_ddot + (struttura.m_a*x_g_ddot)*struttura.R*sin(settings.servo.maxAngle-alpha-theta);
C_aero =  F_A*struttura.R*sin(settings.servo.maxAngle-alpha-theta);
C_load = 3*(C_inertia + C_aero);
% multiplied by three returns the total counter torque

torque_plot = figure;
subplot(1,3,1)
fsurf(C_inertia,[ 0 T_interval 0 V_max])
grid on;
title('Component of torque due to inertia forces w.r.t. time',Interpreter='latex')
xlabel('$t [s]$',Interpreter='latex')
ylabel('$V [m/s]$',Interpreter='latex')
zlabel('$C_{inertia} [N m]$',Interpreter='latex')

subplot(1,3,2)
fsurf(C_aero,[0 V_max 0 T_interval])
grid on;
title('Component of torque due to aerodynamic forces w.r.t. time',Interpreter='latex')
ylabel('$t [s]$',Interpreter='latex')
xlabel('$V [m/s]$',Interpreter='latex')
zlabel('$C_{aero} [N m]$',Interpreter='latex')

subplot(1,3,3)
fsurf(C_load,[0 V_max 0 T_interval])
grid on;
title('Torque load due to air brakes w.r.t. time',Interpreter='latex')
ylabel('$t [s]$',Interpreter='latex')
xlabel('$V [m/s]$',Interpreter='latex')
zlabel('$C_{load} [N m]$',Interpreter='latex')

%% part 3 - motor modeling 
%
% In this section the approximations found up to here are used to propose a
% linear model of the system without using a proper linearization method,
% because the system here is linear.
%
% quick recap on the parameters:
%
% theta is not anymore an arctangent but is linear as follows:
% theta = -0.719817673635389 * alpha + 1.367283470605413
%
% x_g is not anymore a fourth order polynomial but is linear as follows:
% x_g = 0.0308 * alpha
%
% S remains S as it is already linear:
% S = 0.0096 * alpha
%
% Cd is strongly non linear, but for our purpose can be taken as a constant
% value corresponding to the worst case scenario (Cd_max)
% Cd = Cd_max = 1.36; (actually a little bit less, 1.358)
%
% F_A is dependant also on V, the controller can be made in three main
% ways:
% 1) fixed V worst case scenario (simplest solution, not quite accurate)
% 2) gain scheduled for different regimes (generation of a look up table, 
% potentially better and almost fast as the first way) 
% 3) V is considered a variable state that enters the inner loop as a
% "constant" and the transfer function is calculated at each step. (most
% expensive but most accurate solution - more accurate if the speed estimate is
% accurate)
%
% for the purpose of this section it is considered worst case scenario,
% because we need to check the capability of the servo motor to do its
% work.

% creating a struct for geometrical coefficients so we have them all in one place to
% look up to:
% iCoeffs.


%% save:
save = "no";

if save == "yes"
    folder = "digitalTwinFigures";
    mkdir(folder)
saveas(theta_fig,folder+"\theta_fig.png")
saveas(theta_interp1_fig,folder+"\theta_interp1_fig.png")
saveas(theta_linear_fig,folder+"\theta_linear_fig.png")
saveas(x_g_fig,folder+"\x_g_fig.png")
saveas(x_g_interp1_fig,folder+"\x_g_interp1_fig.png")
saveas(x_g_linear_fig,folder+"\x_g_linear_fig.png")
saveas(S_fig,folder+"\S_fig.png")
saveas(Cd_fig,folder+"\Cd_fig.png")
saveas(sin_fig,folder+"\sin_fig.png")
saveas(torque_plot,folder+"\torque_plot.png")
saveas(torque_plot,folder+"\torque_plot.fig")

end
