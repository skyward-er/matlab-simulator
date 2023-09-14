function mach = getMach(v, z)

T0   = 288.15;
alfa= -3.871e-3;
T = T0 +alfa.*z;
c = sqrt(1.4*287.*T);
mach = v./c;

end
