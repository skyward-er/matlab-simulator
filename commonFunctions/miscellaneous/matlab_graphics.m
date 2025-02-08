%{
Set graphical values for better looking plots
%}

%% interpreter:
set(0, 'defaultTextInterpreter', 'latex')
set(0, 'defaultAxesTickLabelInterpreter', 'latex')
set(0, 'defaultLegendInterpreter', 'latex')

%% figure properties:
%colors
set(0, 'defaultFigureColormap',turbo(256));
set(0, 'defaultFigureColor', [1; 1; 1]);
% grid
set(0, 'defaultAxesXGrid','on')
set(0, 'defaultAxesYGrid','on')
% position and scale
screenSize = get(0, 'ScreenSize');
set(0, 'DefaultFigurePosition', [0.3*screenSize(3), 0.25*screenSize(4), 0.4*screenSize(3), 5/6*0.4*screenSize(3)]);
% set(0, 'DefaultFigurePosition', [100,100,600,400]);

%% surfaces:
% transparency
set(0, 'defaultSurfaceEdgeAlpha', 0.3);

%% lines:
defaultLineWidth = 1.5;
% plots
set(0,'defaultLineLineWidth', defaultLineWidth);
% stairs
set(0,'defaultStairLineWidth', defaultLineWidth); % needs a different command for no reason apparently
% ylines
% set(0, 'defaultYLineLineWidth', defaultLineWidth)

%% legend:
set(0, 'defaultLegendLocation','best');
set(0, 'defaultLegendFontSize',7);

%% axes:
% grid 
% set(0, 'defaultAxesXMinorGrid','on','defaultAxesXMinorGridMode','manual');
% set(0, 'defaultAxesYMinorGrid','on','defaultAxesYMinorGridMode','manual');
% font
set(0, 'defaultAxesFontName', 'Palatino Linotype', 'defaultTextFontName', 'Palatino Linotype');

% color
set(0, 'defaultAxesColor', 'none');
% fontSize
set(0,'defaultAxesFontSize',16);

