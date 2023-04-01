function [value, isterminal, direction] = eventSpeed(t, Y,settings, varargin)

 Mach = getMach(norm(Y(4:6)), -Y(3) + settings.z0);
    
value = Mach - 0.8;

isterminal = 1;

direction = 0;

end