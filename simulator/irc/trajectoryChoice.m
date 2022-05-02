function [alpha0]=trajectoryChoice(y0,v_speed,altitude)
%% find closest vertical speed for every reference
distance_vect = zeros(11,1);

z = y0(1);
v = y0(2);

for traj = 1:11 
    [~,j] = min( abs(v-v_speed{traj}) );
    altitude{traj} = altitude{traj}(j:end);
end

%% Find the reference trajectory closed to the initial measured states 
for j = 1:11
    distance_vect(j) = abs(z-altitude{j}(1));
end
[~,i] = min(distance_vect);

%% Save reference data
alpha0 = deg2rad(68)*(i-1)/10;
end