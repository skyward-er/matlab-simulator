function [] = makeCone(pos_cone, radius, z_coord, name)
    % pos_cone: position x, y
    % z_coord: ned position Z every 10 meters (vector)
    % name: string to display in the legend
    if size(z_coord,2)>1
        z_coord = z_coord';
    end
    th = linspace(0, 2*pi, 20);
    X = pos_cone(1);
    Y = pos_cone(2);
    R_circ_dim = linspace(0,radius,z_coord/10+1);
    R_circ = R_circ_dim';
    X_cone = X + R_circ.*cos(th);
    Y_cone = Y + R_circ.*sin(th);
    z_vec = (0:10:z_coord)';
    [~,Z_cone] = meshgrid(th,abs(z_vec));
    surf(X_cone,Y_cone,Z_cone,'FaceAlpha',0.2,'EdgeColor','texturemap','HandleVisibility','off')
    scatter(pos_cone(1), pos_cone(2),'fill', 'DisplayName', name)
end