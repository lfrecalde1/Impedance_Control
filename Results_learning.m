%% Learning results
% clean variables of the system 
clc, clear all, close all;

% Load Data of the system
load("Data_Learning.mat")
qd = Hd;
t = t_aux;
% Change dimentions in the variables
q = q(:, 1:end-1);

% Link 1
l1 = L1(3);
m1 = L1(2);
m1r = 0.1*sqrt(m1);

% Link 2
l2 = L2(3);

% Desired Position on x-y
xd11 = l1*sin(qd(1,1));
yd11 = -l1*cos(qd(1,1));

xd12 =  l2*sin(qd(1, 1)+qd(2, 1)) + l1*sin(qd(1, 1));
yd12 = -l2*cos(qd(1, 1)+qd(2, 1)) - l1*cos(qd(1, 1));

% Real values t = 0
x11 = l1*sin(q(1,1));
y11 = -l1*cos(q(1,1));

x12 = l2*sin(q(1, 1)+q(2, 1)) + l1*sin(q(1, 1));
y12 = -l2*cos(q(1, 1)+q(2, 1)) - l1*cos(q(1, 1));

% Real values t = 1
aux_time_2 = (t >= 2) & (t<2.005);

x21 = l1*sin(q(1,aux_time_2));
y21 = -l1*cos(q(1,aux_time_2));

x22 = l2*sin(q(1, aux_time_2)+q(2, aux_time_2)) + l1*sin(q(1, aux_time_2));
y22 = -l2*cos(q(1, aux_time_2)+q(2, aux_time_2)) - l1*cos(q(1, aux_time_2));

xd21 = l1*sin(qd(1,aux_time_2));
yd21 = -l1*cos(qd(1,aux_time_2));

xd22 =  l2*sin(qd(1, aux_time_2)+qd(2, aux_time_2)) + l1*sin(qd(1, aux_time_2));
yd22 = -l2*cos(qd(1, aux_time_2)+qd(2, aux_time_2)) - l1*cos(qd(1, aux_time_2));

% Real values t =10
aux_time_3 = (t >= 29) & (t<29.005);

x31 = l1*sin(q(1,aux_time_3));
y31 = -l1*cos(q(1,aux_time_3));

x32 = l2*sin(q(1, aux_time_3)+q(2, aux_time_3)) + l1*sin(q(1, aux_time_3));
y32 = -l2*cos(q(1, aux_time_3)+q(2, aux_time_3)) - l1*cos(q(1, aux_time_3));

xd31 = l1*sin(qd(1,aux_time_3));
yd31 = -l1*cos(qd(1,aux_time_3));

xd32 =  l2*sin(qd(1, aux_time_3)+qd(2, aux_time_3)) + l1*sin(qd(1, aux_time_3));
yd32 = -l2*cos(qd(1, aux_time_3)+qd(2, aux_time_3)) - l1*cos(qd(1, aux_time_3));


% Aux time 
t_aux_torque = (t >= 10) & (t < 40);

% Colors and size of the letters
lw = 1.5; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 11; %11
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1250; % size figure
sizeY = 650; % size figure

% color propreties
c1 = [80, 81, 79]/255;
c2 = [244, 213, 141]/255;
c3 = [242, 95, 92]/255;
c4 = [112, 141, 129]/255;
c5 = [ 254, 217, 217]/255;

C18 = [0 0 0];

% Create Figure
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

%% Data generation
axes('Position',[0.1 0.60 .22 .36]);

%% Desired  system plot
link_1_plot_t_0 = line([0 xd11],[0 yd11]);
link_2_plot_t_0 = line([xd11 xd12],[yd11 yd12]);

rectangle('Position',[0-m1r/2 0-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c1);
rectangle('Position',[xd11-m1r/2 yd11-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c1);
rectangle('Position',[xd12-m1r/2 yd12-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c1);

set(link_1_plot_t_0, 'LineStyle', '-', 'Color', c1, 'LineWidth', lw*1.5)
set(link_2_plot_t_0, 'LineStyle', '-', 'Color', c1, 'LineWidth', lw*1.5)


%% Real system plot
l_1_real_t_0 = line([0 x11],[0 y11]);
l_2_real_t_0 = line([x11 x12],[y11 y12]);

rectangle('Position',[x11-m1r/2 y11-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c3);
rectangle('Position',[x12-m1r/2 y12-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c3);

set(l_1_real_t_0, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)
set(l_2_real_t_0, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)


%% Title of the image
hTitle_1 = title({'$t = 0[s]$'},'fontsize',14,'interpreter','latex','Color',C18);
hXLabel_1 = xlabel('$x[m]$','fontsize',10,'interpreter','latex', 'Color',C18);
hYLabel_1 = ylabel('$y[m]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_1 = legend([link_1_plot_t_0, l_1_real_t_0],{'$\textrm{Reference}$','$\textrm{Real}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.02;0.02];
ax_1.TickDirMode = 'auto';
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;
ax_1.XLim = [-2.1 2.1];
ax_1.YLim = [-2.1 2.1];

%% Data generation
axes('Position',[0.35 0.6 .22 .36]);

%% Desired  system plot
link_1_plot_t_30 = line([0 xd21],[0 yd21]);
link_2_plot_t_30 = line([xd21 xd22],[yd21 yd22]);

rectangle('Position',[0-m1r/2 0-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c1);
rectangle('Position',[xd21-m1r/2 yd21-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c1);
rectangle('Position',[xd22-m1r/2 yd22-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c1);

set(link_1_plot_t_30, 'LineStyle', '-', 'Color', c1, 'LineWidth', lw*1.5)
set(link_2_plot_t_30, 'LineStyle', '-', 'Color', c1, 'LineWidth', lw*1.5)


%% Real system plot
l_1_real_t_30 = line([0 x21],[0 y21]);
l_2_real_t_30 = line([x21 x22],[y21 y22]);

rectangle('Position',[x21-m1r/2 y21-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c3);
rectangle('Position',[x22-m1r/2 y22-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c3);

set(l_1_real_t_30, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)
set(l_2_real_t_30, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)


%% Title of the image
hTitle_ = title({'$t = 2[s]$'},'fontsize',14,'interpreter','latex','Color',C18);
hXLabel_2 = xlabel('$x[m]$','fontsize',10,'interpreter','latex', 'Color',C18);
%hYLabel_2 = ylabel('$y[m]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_2 = legend([link_1_plot_t_30, l_1_real_t_30],{'$\textrm{Reference}$','$\textrm{Real}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
% Figure properties
ax_2 = gca;
ax_2.Box = 'on';
ax_2.BoxStyle = 'full';
ax_2.TickLength = [0.02;0.02];
ax_2.TickDirMode = 'auto';
ax_2.YMinorTick = 'on';
ax_2.XMinorTick = 'on';
ax_2.XMinorGrid = 'on';
ax_2.YMinorGrid = 'on';
ax_2.MinorGridAlpha = 0.15;
ax_2.LineWidth = 0.8;
ax_2.XLim = [-2.1 2.1];
ax_2.YLim = [-2.1 2.1];

%% Data generation
axes('Position',[0.6 0.6 .22 .36]);

%% Desired  system plot
link_1_plot_t_60 = line([0 xd31],[0 yd31]);
link_2_plot_t_60 = line([xd31 xd32],[yd31 yd32]);

rectangle('Position',[0-m1r/2 0-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c1);
rectangle('Position',[xd31-m1r/2 yd31-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c1);
rectangle('Position',[xd32-m1r/2 yd32-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c1);

set(link_1_plot_t_60, 'LineStyle', '-', 'Color', c1, 'LineWidth', lw*1.5)
set(link_2_plot_t_60, 'LineStyle', '-', 'Color', c1, 'LineWidth', lw*1.5)


%% Real system plot
l_1_real_t_60 = line([0 x31],[0 y31]);
l_2_real_t_60 = line([x31 x32],[y31 y32]);

rectangle('Position',[x31-m1r/2 y31-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c3);
rectangle('Position',[x32-m1r/2 y32-m1r/2 m1r m1r],'Curvature',1,'FaceColor',c3);

set(l_1_real_t_60, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)
set(l_2_real_t_60, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)


%% Title of the image
hTitle_ = title({'$t = 25[s]$'},'fontsize',14,'interpreter','latex','Color',C18);
hXLabel_3 = xlabel('$x[m]$','fontsize',10,'interpreter','latex', 'Color',C18);
%hYLabel_3 = ylabel('$y[m]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([link_1_plot_t_60, l_1_real_t_60],{'$\textrm{Reference}$','$\textrm{Real}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
% Figure properties
ax_3 = gca;
ax_3.Box = 'on';
ax_3.BoxStyle = 'full';
ax_3.TickLength = [0.02;0.02];
ax_3.TickDirMode = 'auto';
ax_3.YMinorTick = 'on';
ax_3.XMinorTick = 'on';
ax_3.XMinorGrid = 'on';
ax_3.YMinorGrid = 'on';
ax_3.MinorGridAlpha = 0.15;
ax_3.LineWidth = 0.8;
ax_3.XLim = [-2.1 2.1];
ax_3.YLim = [-2.1 2.1];


%hold on;
axes('Position',[0.1 0.30 .72 .21]);
% Aux time
hightlighting = t(t_aux_torque);
number_faces = [1 2 3 4];
vertices = [hightlighting(1) -2; hightlighting(1) 2; hightlighting(end) 2; hightlighting(end) -2];
patch('Faces',number_faces,'Vertices',vertices,'FaceColor',c5,'FaceAlpha',.2, 'Marker','.');
%% Data generation
error1_plot = line(t(1:length(qe)),qe(1,:));
set(error1_plot, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw);
error2_plot = line(t(1:length(qe)),qe(2,:));
set(error2_plot, 'LineStyle', '-', 'Color', c4, 'LineWidth', lw);
text(20,1,'$\textrm{Perturbacion Extra}$','interpreter','latex','fontsize',12);
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Error}[rad]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_1 = legend([error1_plot,error2_plot],{'$\tilde{{q}}_1$','${\tilde{{q}}}_2$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.XTickLabel = [];
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t(end-1)];

%hold on;
axes('Position',[0.1 0.07 .72 .21]);
%% Data generation
control1_plot = line(t(1:length(u)),u(1,:));
set(control1_plot, 'LineStyle', '-', 'Color', c4, 'LineWidth', lw);
control2_plot = line(t(1:length(u)),u(2,:));
set(control2_plot, 'LineStyle', '-', 'Color', c2, 'LineWidth', lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Control}[N.m]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_5 = legend([control1_plot,control2_plot],{'$\tau_1$','$\tau_2$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_5 = gca;
ax_5.Box = 'on';
ax_5.BoxStyle = 'full';
ax_5.TickLength = [0.01;0.01];
ax_5.TickDirMode = 'auto';
ax_5.YMinorTick = 'on';
ax_5.XMinorTick = 'on';
ax_5.XMinorGrid = 'on';
ax_5.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_5.MinorGridAlpha = 0.15;
ax_5.LineWidth = 0.8;
ax_5.XLim = [0 t(end-1)];

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Results_learning.pdf -q101
