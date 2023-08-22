% Author: Pier Francesco Bachini
%
% This function displays a parallelepiped which has its orientation in space
% represented by the provided quaternions.
% If the quaternions are vectors all of the same length, then the
% parallelepid orientation will be animated following the quaternion data.
% NOTE: The quaternion data must be provided in x,y,z,w order

function animateOrientation(qx_vect, qy_vect, qz_vect, qw_vect, time, video_name)
      
    if ~(isvector(qx_vect) && isvector(qy_vect) && isvector(qz_vect) && isvector(qw_vect))
        error("All quaternions must be vectors or scalar");
    end
    if (numel(qx_vect) ~= numel(qy_vect) || numel(qy_vect) ~= numel(qz_vect) ...
            || numel(qz_vect) ~= numel(qw_vect))
        error("All quaternions must have the same length");
    end

    connections = [4 8 5 1 4;
                   1 5 6 2 1;
                   2 6 7 3 2; 
                   3 7 8 4 3; 
                   5 8 7 6 5; 
                   1 4 3 2 1]';    
    
    colors = [0         0.4470  0.7410;
              0.8500    0.3250  0.0980;
              0.9290    0.6940  0.1250;
              0.4940    0.1840  0.5560;
              0.4660    0.6740  0.1880;
              0.3010    0.7450  0.9330];

    figure();
    view(3);
    hold on;
    axis equal
    axis([-2 2 -2 2 -2 2]);
    view(0,0.1);
    plot3([-2 2], [0 0], [0 0], 'Color', 'blue'); % blue x
    plot3([0 0], [-2 2], [0 0], 'Color', 'magenta'); % orange y
    plot3([0 0], [0 0], [-2 2], 'Color', 'green'); % green z

    % Time box settings
    time_elapsed = time - time(1); % Set new t0 = 0
    timeBoxLocation = [.35 .8 .1 .1];
    timeBox = annotation('textbox',timeBoxLocation,'String', time_elapsed(1),  'Interpreter', 'latex');
    timeBox.FontSize = 24;
    timeBox.BackgroundColor= 'w';

    if nargin > 5 %Set video settings
        % High frame-rate for video
        frame_rate = 5;
    
        % Change renderer
        set(gcf, 'renderer', 'zbuffer');
        v = VideoWriter(video_name);
        v.FrameRate = frame_rate;
        open(v);
    else
        % Low frame-rate for only animation
        frame_rate = 2;
    end

    for t = 2:(50/frame_rate):length(qx_vect) % 50 Hz is the frequency of NAS
        qx = qx_vect(t);
        qy = qy_vect(t);
        qz = qz_vect(t);
        qw = qw_vect(t);
        R = quat2dcm([qw qx qy qz]);
        A = R*[-1 -1 -.1]';
        B = R*[ 1 -1 -.1]';
        C = R*[ 1  1 -.1]';
        D = R*[-1  1 -.1]';
        E = R*[-1 -1 .1]';
        F = R*[ 1 -1 .1]';
        G = R*[ 1  1 .1]';
        H = R*[-1  1 .1]';
    
        vertices = [A';B';C';D';E';F';G';H'];
        xc = vertices(:,1);
        yc = vertices(:,2);
        zc = vertices(:,3);
    
        if isempty(findobj('type', 'patch'))
            p = patch(xc(connections), yc(connections), zc(connections), 'r', 'facealpha', 1, 'FaceVertexCData', colors, 'FaceColor', 'Flat'); 
        else
            p.XData = xc(connections);
            p.YData = yc(connections);
            p.ZData = zc(connections);
        end
        formatSpec = '%.3f';
        sec = num2str(time_elapsed(t),formatSpec);
        timeBox.String = sprintf('% s', sec);
        drawnow
        pause(0.02);

        if nargin > 5
	        thisFrame = getframe(gcf);
	        writeVideo(v, thisFrame);
        end
    end

    if nargin > 5
        close(v);
    end
end