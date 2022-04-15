function [x] = extension_From_Angle_2022(alpha,settings)

x = settings.extPol(1)*alpha.^4 + settings.extPol(2)*alpha.^3+settings.extPol(3)*alpha.^2 + settings.extPol(4).*alpha;

end