
for ii = 1:length(trajectories_saving)

    trajectories_obsw{ii} = [trajectories_saving{ii}.Z_ref,trajectories_saving{ii}.VZ_ref];

end
for ii = 1:length(trajectories_saving)
    filename = ['saving\Trajectory','_',num2str(ii-1)];
    writematrix(trajectories_obsw{1,ii},[filename,'.csv']);

end