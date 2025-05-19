clc
clear
close all
load('Environment_data.mat')
global F G parent
%% 绘制障碍物地图、起始点、目标点
load('Environment_data.mat')
figure(1)
set(gcf,'position',[200,200,600,600]);
axis([0 col*delta 0 row*delta 0 ceng*delta])
hold on
plot3(S(1),S(2),S(3), 'o','markersize', 8,'markerfacecolor','g','MarkerEdgeColor', 'k')
plot3(E(1),E(2),E(3), 'o','markersize', 8,'markerfacecolor','r','MarkerEdgeColor', 'k')
shp = alphaShape(ob_coo(:,2),ob_coo(:,1),ob_coo(:,3));
plot(shp,'EdgeColor','k','FaceColor','k','FaceAlpha',0.5);
set(gca,'FontSize',16,'Fontname', 'Times New Roman');
% view(-37.5,30)
view(-41,45)
notCross = 1; % M中不能通过的点的值
F = zeros(row,col,ceng);%初始化F矩阵
G = zeros(row,col,ceng);%初始化G矩阵
mov = [1 0 0;1 0 1;1 0 -1;1 1 0;1 1 1;1 1 -1;1 -1 0;1 -1 1;1 -1 -1;
       0 0 1;0 0 -1;0 1 0;0 1 1;0 1 -1;0 -1 0;0 -1 1;0 -1 -1;
       -1 0 0;-1 0 1;-1 0 -1;-1 1 0;-1 1 1;-1 1 -1;-1 -1 0;-1 -1 1;-1 -1 -1;];%26个移动方向
opened= [A];%开放列表
closed = [];%关闭列表
parent = zeros(row,col,ceng);
inOpen = false(row,col,ceng);   
inClose = false(row,col,ceng);  
%% ================================================
%
inOpen(A(1),A(2),A(3)) = true;
G(A(1),A(2),A(3)) = 0;
F(A(1),A(2),A(3)) = hn(A,B);
[index,min_p] = minInOpen(opened);
before_p=[];
tic
while min_p(1)~=B(1)||min_p(2)~=B(2)||min_p(3)~=B(3)  
    opened(index,:) = [];     %从开放列表中删除
    inOpen(min_p(1), min_p(2), min_p(3)) = false;
    closed = [closed;min_p];
    inClose(min_p(1), min_p(2), min_p(3)) = true;
    for i=1:size(mov,1)
        temp = min_p + mov(i,:);
        if temp(1)<=row&&temp(1)>0&&temp(2)<=col&&temp(2)>0&&temp(3)<=ceng&&temp(3)>0
            if sign(temp(1),temp(2),temp(3)) ~= notCross&&inClose(temp(1),temp(2),temp(3)) == false
%     plot3((temp(2)-0.5)*delta,(temp(1)-0.5)*delta,(temp(3)-0.5)*delta, 'o','markersize', 4,'markerfacecolor','b','MarkerEdgeColor', 'b')
% 	drawnow     
                if inOpen(temp(1),temp(2),temp(3)) == 0
                    parent(temp(1),temp(2),temp(3)) = sub2ind([row,col,ceng],min_p(1),min_p(2),min_p(3));
                    opened = [opened;temp];
                    G(temp(1),temp(2),temp(3)) = gn(temp);
                    F(temp(1),temp(2),temp(3)) = G(temp(1),temp(2),temp(3))+hn(temp,B);
                    inOpen(temp(1), temp(2), temp(3)) = true;  
                else           % 在开放列表中
                   gnn = norm(min_p-temp) + G(min_p(1),min_p(2),min_p(3)); %
                    if gnn < G(temp(1),temp(2),temp(3))
                        parent(temp(1),temp(2),temp(3)) = sub2ind([row,col,ceng],min_p(1),min_p(2),min_p(3));
                        G(temp(1),temp(2),temp(3))=gnn;
                        F(temp(1),temp(2),temp(3)) = G(temp(1),temp(2),temp(3))+hn(temp,B);
                    end
                end
            end
        end
    end
    before_p=min_p;
    [index,min_p] = minInOpen(opened);
    a=plot3((min_p(2)-0.5)*delta,(min_p(1)-0.5)*delta,(min_p(3)-0.5)*delta, 'o','markersize', 3,'markerfacecolor','g','MarkerEdgeColor', 'g');
    set(a,'handlevisibility','off');
end
    set(a,'handlevisibility','on');
T_all=toc;
route =[B];
t=parent(B(1),B(2),B(3));
indA =sub2ind([row,col,ceng],A(1),A(2),A(3));
while t ~=  indA
    [pc(1),pc(2),pc(3)]=ind2sub([row,col,ceng],t);
    route = [route;pc];
    t=parent(t);
end
route = [route;A];
route=flipud(route);
Astar_route=(route-0.5)*delta;
Astar_route(:,1:2)=fliplr(Astar_route(:,1:2));
plot3(Astar_route(:,1),Astar_route(:,2),Astar_route(:,3),'b','LineWidth',2)
xlim([0 500]);
ylim([0 500]);
legend('Source','Goal','Obstacles','closeList','Path')
% view(-37.5,30)
view(-41,45)
disp(['A*算法相关结果：'])
disp(['程序运行时间：',num2str(T_all)])

for i=2:size(Astar_route(:,1))
    Q1=Astar_route(i-1,:);
    Q2=Astar_route(i,:);
dis(i)=((Q1(1)-Q2(1)).^2+(Q1(2)-Q2(2)).^2+(Q1(3)-Q2(3)).^2).^0.5;
end
disp(['Path length:',num2str(sum(dis,'all'))]);
disp(' Path as follows: (from S to E)');
disp(Astar_route);

%% 去除多余节点
u=1;
o=0;
uuu=[]; 
Astar_route_simpify=[S];
while o < size(Astar_route,1) 
  for uu=u:size(Astar_route,1)
    if checkPath(Astar_route(u,1:3),Astar_route(uu,1:3), shp)
        uuu=[uuu;uu]; %用来存储所有可以到达的点
    end 
  end
     o=uuu(end);%uuu中的最后一个节点
     Astar_route_simpify = [Astar_route_simpify;Astar_route(o,:)];
    u=o;
end


figure(2)
axis([0 col*delta 0 row*delta 0 ceng*delta])
hold on
set(gca,'FontSize',16,'Fontname', 'Times New Roman');
set(gcf,'position',[200,200,600,600]);
plot3(S(1),S(2),S(3), 'o','markersize', 8,'markerfacecolor','g','MarkerEdgeColor', 'k')
plot3(E(1),E(2),E(3), 'o','markersize', 8,'markerfacecolor','r','MarkerEdgeColor', 'k')
shp = alphaShape(ob_coo(:,2),ob_coo(:,1),ob_coo(:,3));
plot(shp,'EdgeColor','k','FaceColor','k','FaceAlpha',0.5);
plot3(Astar_route(:,1),Astar_route(:,2),Astar_route(:,3),'b--','LineWidth',2)
plot3(Astar_route_simpify(:,1),Astar_route_simpify(:,2),Astar_route_simpify(:,3),'k','LineWidth',2); 
a=plot3(Astar_route_simpify(:,1),Astar_route_simpify(:,2),Astar_route_simpify(:,3),'o','markersize', 5,'markerfacecolor',[0 .7 .7],'MarkerEdgeColor',[0 .7 .7]); 
if Astar_route_simpify(1,1) ~= S(1,1)
set(a,'handlevisibility','on');
end
xlim([0 500]);
ylim([0 500]);
legend('Source','Goal','Obstacles','Astar-path','Astar-path simpify','Key point')
% view(-37.5,30)
view(-41,45)
save Global_path Astar_route Astar_route_simpify