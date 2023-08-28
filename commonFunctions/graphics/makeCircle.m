function [] = makeCircle(pos, radius, varargin)
    % pos_cone: position x, y
    % z_coord: ned position Z every 10 meters (vector)
    % name: string to display in the legend

    % include options
    p = inputParser;
    
    % add default parameters
    addParameter(p, 'DisplayName','');
    addParameter(p, 'Alpha',1);
    addParameter(p, 'FaceColor',1);

    % parse option inputs
    parse(p, varargin{:});

    % recall options
    name        = p.Results.DisplayName;
    alpha       = p.Results.Alpha;
    color       = p.Results.FaceColor;
    % draw circle
    N = 100;
    th = linspace(0, 2*pi, N);
    r_coord = (linspace(0,radius,N))';
    x = pos(1) + r_coord.*cos(th);
    y = pos(2) + r_coord.*sin(th);
    % [X,Y] = meshgrid(x,y); 

    surf(x,y,zeros(size(x)),'FaceAlpha',alpha,'DisplayName', name,'EdgeColor','none','FaceColor',color)
end