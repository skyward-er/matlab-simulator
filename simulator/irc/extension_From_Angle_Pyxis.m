function [x] = extension_From_Angle_Pyxis(alpha,settings)

x = settings.arb.extPol(1)*alpha.^4 + settings.arb.extPol(2)*alpha.^3+settings.arb.extPol(3)*alpha.^2 + settings.arb.extPol(4).*alpha;

end