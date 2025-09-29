function mach = getMach(v, z)

T0  = 288.15;
a   = -0.0065;
T   = T0 + a .* z;
c = sqrt(1.4 * 287.05 .* T);
mach = v ./ c;

end
