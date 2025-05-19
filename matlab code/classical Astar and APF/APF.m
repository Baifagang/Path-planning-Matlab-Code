%% ===========
% row: 行
% col: 列
% ceng: 高
% 栅格地图尺寸:100（行）*100（列）*24（高）
% delta:单元栅格长度为5
%     A:初始点[行,列,高]
%         S:初始点坐标值
%     B:目标点[行,列,高]
%         E:目标点坐标值
%     注意：实际上行对应y，列对应x    
% ob_num:障碍物节点个数
% ob_r:单元栅格对角线长度的一半:sqrt(3)/2*delta
% ob_coo:障碍物[行,列,高]*delta
%% ===========
clc
clear
close all
load('Environment_data.mat')  % 加载地图信息
load('Global_path.mat')       % 加载全局路径信息
figure(1)
set(gcf,'position',[200,200,600,600]);
axis([0 col*delta 0 row*delta 0 ceng*delta])
hold on
plot3(S(1),S(2),S(3), 'o','markersize', 8,'markerfacecolor','g','MarkerEdgeColor', 'k')
plot3(E(1),E(2),E(3), 'o','markersize', 8,'markerfacecolor','r','MarkerEdgeColor', 'k')
shp = alphaShape(ob_coo(:,2),ob_coo(:,1),ob_coo(:,3));
plot(shp,'EdgeColor','k','FaceColor','k','FaceAlpha',0.5);
plot3(Astar_route(:,1),Astar_route(:,2),Astar_route(:,3),'b--','LineWidth',1.5);
plot3(Astar_route_simpify(:,1),Astar_route_simpify(:,2),Astar_route_simpify(:,3),'k--','LineWidth',2);
% plot3(route_smart(:,1),route_smart(:,2),route_smart(:,3),'r--','LineWidth',2);
set(gca,'FontSize',16,'Fontname', 'Times New Roman');


T_num=1;%计时
step=2;%调用人工势场时的步长
dis_purpose=step;%当小车与目标点距离小于该值时，即认为到达目标
K1=10;%引力增益系数
K2=50;%斥力增益系数
K3=0.2;%偏航引力系数
ob_coo(:,1:2)=fliplr(ob_coo(:,1:2));
safe_dis=10*ob_r;
now_point=S;
end_point=E;
APF_route2=now_point;%新路径的初始起点
tic
while true %循环开始
    Attract_x=K1*(end_point(1)-now_point(1));        %x方向上的引力
    Attract_y=K1*(end_point(2)-now_point(2));        %y方向上的引力
    Attract_z=K1*(end_point(3)-now_point(3));        %y方向上的引力
    Attract_x=Attract_x/norm(end_point-now_point);
    Attract_y=Attract_y/norm(end_point-now_point);
    Attract_z=Attract_z/norm(end_point-now_point);
            repulsion_x=0;                          %x方向上的斥力
            repulsion_y=0;                          %y方向上的斥力
            repulsion_z=0;                          %z方向上的斥力
    for i=1:ob_num
        dis_lin=norm(now_point-ob_coo(i,1:3));      %路径点和障碍的距离
        if dis_lin<safe_dis                         %如果障碍和当前点的距离小于障碍影响距离          
            repulsion_x = repulsion_x + K2*(1/dis_lin-1/safe_dis)*(1/dis_lin) *(ob_coo(i,1)-now_point(1))/dis_lin;
            repulsion_y = repulsion_y + K2*(1/dis_lin-1/safe_dis)*(1/dis_lin) *(ob_coo(i,2)-now_point(2))/dis_lin;
            repulsion_z = repulsion_z + K2*(1/dis_lin-1/safe_dis)*(1/dis_lin) *(ob_coo(i,3)-now_point(3))/dis_lin;
        end
    end

    [yaw_p,dis]=yaw_fun(now_point,Astar_route_simpify);
    Attract_x=Attract_x+(yaw_p(1)-now_point(1))*dis*K3;
    Attract_y=Attract_y+(yaw_p(2)-now_point(2))*dis*K3;
    Attract_z=Attract_z+(yaw_p(3)-now_point(3))*dis*K3;
    sum_x=Attract_x-repulsion_x;%x方向的合力
    sum_y=Attract_y-repulsion_y;%y方向的合力
    sum_z=Attract_z-repulsion_z;%z方向的合力
    next_Point(1)=now_point(1)+step*sum_x/(sum_x^2+sum_y^2+sum_z^2)^0.5;
    next_Point(2)=now_point(2)+step*sum_y/(sum_x^2+sum_y^2+sum_z^2)^0.5;
    next_Point(3)=now_point(3)+step*sum_z/(sum_x^2+sum_y^2+sum_z^2)^0.5;
    
    plot3([now_point(1) next_Point(1)],[now_point(2) next_Point(2)],[now_point(3) next_Point(3)],'m','LineWidth',2)
    
    now_point=next_Point;
    T_num=T_num+1;
    APF_route2(T_num,:)=now_point;
    dis_Goal=norm(now_point-end_point);%计算当前点与目标点距离
    if dis_Goal<dis_purpose %距离小于给定值，则认为到达目标点
        T_num=T_num+1;
        APF_route2(T_num,:)=end_point;%把终点添加到路径
        break;
    end
end
T_all=toc;
disp(['程序运行时间：',num2str(T_all)])
for j=2:size(APF_route2(:,1))
    Q3=APF_route2(j-1,:);
    Q4=APF_route2(j,:);
dis2(j)=((Q3(1)-Q4(1)).^2+(Q3(2)-Q4(2)).^2+(Q3(3)-Q4(3)).^2).^0.5;
end
disp(['APF Path length:',num2str(sum(dis2,'all'))]);
disp('APF Path as follows: (from S to E)');
disp(APF_route2);