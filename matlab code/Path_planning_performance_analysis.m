clear,clc
figure(1)
x = categorical({'(5,3,2)/(85,93,8)','(85,93,8)/(5,3,2)','(10,13,2)/(63,64,8)',...
    '(63,64,8)/(10,13,2)','(2,35,24)/(92,5,8)','(92,5,8)/(2,35,24)'});
x = reordercats(x,{'(5,3,2)/(85,93,8)','(85,93,8)/(5,3,2)','(10,13,2)/(63,64,8)',...
    '(63,64,8)/(10,13,2)','(2,35,24)/(92,5,8)','(92,5,8)/(2,35,24)'});
%% 路径长度
% % 地图1
% vals = [621.212 618.160 411.239 416.504 484.103 483.484;621.193 616.976 410.695 410.758 489.103 484.397];
% 地图2
% vals = [630.427 649.150 429.157 469.930 481.638 481.690;631.010 624.891 420.001 418.105 481.701 481.742];
%% 节点数量
% 地图1
% vals = [1532 1254 1023 1003 1195 1218;711 537 445 405 459 473];
% % 地图2
% vals = [3918 3382 2941 1063 1827 1070;2900 2936 951 398 614 437];
%% 搜索时间
% 地图1
vals = [4.493 4.225 2.765 2.893 3.235 3.293;4.296 4.401 3.021 2.979 3.127 3.229];
% % 地图2
% vals = [38.833 32.379 16.964 8.345 9.067 8.444;19.732 17.401 8.534 7.340 7.915 7.181];
b = bar(x,vals);
xtips1 = b(1).XEndPoints;
ytips1 = b(1).YEndPoints;
labels1 = string(b(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom','Fontname', 'Times New Roman','FontSize',12)
xtips2 = b(2).XEndPoints;
ytips2 = b(2).YEndPoints;
labels2 = string(b(2).YData);
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom','Fontname', 'Times New Roman','FontSize',12')
legend('Classic Astar + secondary planning + artificial potential field',...
    'Bidirectional Astar + secondary planning + artificial potential field')
xlabel('Source Point/Goal Point')
% ylabel('Path length(cm)')
% ylabel('Number of expansion nodes')
ylabel('Search time(s)')
% ylim([0 750]);
% ylim([0 4000]);
ylim([0 5.5]);
set(gca,'FontSize',16,'Fontname', 'Times New Roman');
set(gcf,'position',[100 200 1000 500]);