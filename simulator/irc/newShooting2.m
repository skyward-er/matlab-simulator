function [res] = newShooting2(ap,settings,coeffs,k,y0,targ)


v0 = y0(2);
h0 = abs(y0(1));

g = 9.81;
S = (pi/4) * settings.C^2;
m = settings.ms;
% rho = 1.225;
H = 7.249;
rho  = 1.225*exp(y0(1)*1e-3/H);
% [~, ~, ~, rho] = atmosisa(-y(1));
xg = k.extPol(1)*ap^4 + k.extPol(2)*ap^3+k.extPol(3)*ap^2 + k.extPol(4)*ap;

Cd = getDrag(abs(y0(2)),-y0(1),xg,coeffs);

B = 0.5*rho*Cd*S/m;
hf = h0 + (1/(2*B))*log(1+(v0^2)*B/g);

res = targ-hf;
end

    