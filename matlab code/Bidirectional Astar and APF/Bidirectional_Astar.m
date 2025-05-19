clc
clear
close all
%% ====================Bidirectional_Astar==========================================
%% �����ϰ����ͼ����ʼ�㡢Ŀ���
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
view(-37.5,30)
%% ��ʼ��Open��Close�б�
notCross = 1; % ����ͨ���ĵ��ֵ
F1 = zeros(row,col,ceng);%��ʼ��F����
G1 = zeros(row,col,ceng);%��ʼ��G����
F2 = zeros(row,col,ceng);%��ʼ��F����
G2 = zeros(row,col,ceng);%��ʼ��G����
% �ӽڵ��26���ƶ�����
mov = [1 0 0;1 0 1;1 0 -1;1 1 0;1 1 1;1 1 -1;1 -1 0;1 -1 1;1 -1 -1;
       0 0 1;0 0 -1;0 1 0;0 1 1;0 1 -1;0 -1 0;0 -1 1;0 -1 -1;
      -1 0 0;-1 0 1;-1 0 -1;-1 1 0;-1 1 1;-1 1 -1;-1 -1 0;-1 -1 1;-1 -1 -1;];
opened1= [A]; % �������б�
closed1 = []; % ���ر��б�
opened2= [B]; % �������б�
closed2 = []; % ���ر��б�
parent1 = zeros(row,col,ceng);
inOpen1 = false(row,col,ceng);   
inClose1 = false(row,col,ceng);  
parent2 = zeros(row,col,ceng);
inOpen2 = false(row,col,ceng);   
inClose2 = false(row,col,ceng);

inOpen1(A(1),A(2),A(3)) = true;
G1(A(1),A(2),A(3)) = 0;                    % ��ʼʱ��ʵ�ʴ��ۣ�����
F1(A(1),A(2),A(3))= hn(A,B);               % ��ʼʱ��Ԥ�����ۣ�����
inOpen2(B(1),B(2),B(3)) = true;     
G2(B(1),B(2),B(3)) = 0;                    % ��ʼʱ��ʵ�ʴ��ۣ�����
F2(B(1),B(2),B(3))= hn(B,A);               % ��ʼʱ��Ԥ�����ۣ�����
[index1,min_p1] = minInOpen_bid(opened1,F1);  % �����������б���f(n)��С�Ľڵ�
[index2,min_p2] = minInOpen_bid(opened2,F2);  % ���㸺�����б���f(n)��С�Ľڵ�
%% �㷨����
tic % ��ʼ��ʱ
while true
    opened1(index1,:) = [];     %���������б���ɾ��
    opened2(index2,:) = [];     %�Ӹ������б���ɾ��
    inOpen1(min_p1(1), min_p1(2), min_p1(3)) = false;
    inOpen2(min_p2(1), min_p2(2), min_p2(3)) = false;
    closed1 = [closed1;min_p1]; %�������ر��б�
    closed2 = [closed2;min_p2]; %���븺�ر��б�
    inClose1(min_p1(1), min_p1(2), min_p1(3)) = true;
    inClose2(min_p2(1), min_p2(2), min_p2(3)) = true;   
             %������Close���ڽڵ��ͼ
             a=plot3((min_p1(2)-0.5)*delta,(min_p1(1)-0.5)*delta,(min_p1(3)-0.5)*delta, 'o','markersize', 3,'markerfacecolor','g','MarkerEdgeColor', 'g');
             b=plot3((min_p2(2)-0.5)*delta,(min_p2(1)-0.5)*delta,(min_p2(3)-0.5)*delta, 'o','markersize', 3,'markerfacecolor','r','MarkerEdgeColor', 'r');
             set(a,'handlevisibility','off');
             set(b,'handlevisibility','off');
	 for i=1:size(mov,1)
        %% �����жϸ��ڵ��26���ӽڵ�
        temp1 = min_p1 + mov(i,:);
        if temp1(1)<=row&&temp1(1)>0&&temp1(2)<=col&&temp1(2)>0&&temp1(3)<=ceng&&temp1(3)>0
            if sign(temp1(1),temp1(2),temp1(3))~= notCross && inClose1(temp1(1),temp1(2),temp1(3)) == false
%                   plot3((temp1(2)-0.5)*delta,(temp1(1)-0.5)*delta,(temp1(3)-0.5)*delta, 'o','markersize', 4,'markerfacecolor','b','MarkerEdgeColor', 'b')
% 	                drawnow     
                if inOpen1(temp1(1),temp1(2),temp1(3)) == 0
                    parent1(temp1(1),temp1(2),temp1(3)) = sub2ind([row,col,ceng],min_p1(1),min_p1(2),min_p1(3));
                    opened1 = [opened1;temp1];                
                    G1(temp1(1),temp1(2),temp1(3)) = gn_bid(temp1,G1,parent1);
                    F1(temp1(1),temp1(2),temp1(3)) =G1(temp1(1),temp1(2),temp1(3))+hn(temp1,min_p2);
                    inOpen1(temp1(1), temp1(2), temp1(3)) = true;  
                else           % �ڿ����б���
                   gnn1 = norm(min_p1-temp1) + G1(min_p1(1),min_p1(2),min_p1(3)); %
                    if gnn1 < G1(temp1(1),temp1(2),temp1(3))
                        parent1(temp1(1),temp1(2),temp1(3)) = sub2ind([row,col,ceng],min_p1(1),min_p1(2),min_p1(3));
                        G1(temp1(1),temp1(2),temp1(3))=gnn1;
                        F1(temp1(1),temp1(2),temp1(3)) =G1(temp1(1),temp1(2),temp1(3))+hn(temp1,min_p2);
                    end
                end                                  
            end
        end
        %% �����жϸ��ڵ��26���ӽڵ�
        temp2 = min_p2 + mov(i,:);
        if temp2(1)<=row&&temp2(1)>0&&temp2(2)<=col&&temp2(2)>0&&temp2(3)<=ceng&&temp2(3)>0
            if sign(temp2(1),temp2(2),temp2(3)) ~= notCross &&inClose2(temp2(1),temp2(2),temp2(3)) == false
%                   plot3((temp2(2)-0.5)*delta,(temp2(1)-0.5)*delta,(temp2(3)-0.5)*delta, 'o','markersize', 4,'markerfacecolor','b','MarkerEdgeColor', 'b')
% 	                drawnow    
                if inOpen2(temp2(1),temp2(2),temp2(3)) == 0
                    parent2(temp2(1),temp2(2),temp2(3)) = sub2ind([row,col,ceng],min_p2(1),min_p2(2),min_p2(3));
                    opened2 = [opened2;temp2];                 
                    G2(temp2(1),temp2(2),temp2(3)) = gn_bid(temp2,G2,parent2);
                    F2(temp2(1),temp2(2),temp2(3))=G2(temp2(1),temp2(2),temp2(3))+hn(temp2,min_p1);
                    inOpen2(temp2(1), temp2(2), temp2(3)) = true;  
                else           % �ڿ����б���
                   gnn2 = norm(min_p2-temp2) + G2(min_p1(1),min_p2(2),min_p2(3)); %
                    if gnn2 < G2(temp2(1),temp2(2),temp2(3))
                        parent2(temp2(1),temp2(2),temp2(3)) = sub2ind([row,col,ceng],min_p2(1),min_p2(2),min_p2(3));
                        G2(temp2(1),temp2(2),temp2(3))=gnn2;
                        F2(temp2(1),temp2(2),temp2(3))= G2(temp2(1),temp2(2),temp2(3))+hn(temp2,min_p1);
                    end
                end                                     
            end
        end 
	 end

     [index1,min_p1] = minInOpen_bid(opened1,F1);
     [index2,min_p2] = minInOpen_bid(opened2,F2);
    opened_f=sub2ind([row,col,ceng],opened1(:,1), opened1(:,2), opened1(:,3));
    opened_r=sub2ind([row,col,ceng],opened2(:,1), opened2(:,2), opened2(:,3));
    closed_f=sub2ind([row,col,ceng],closed1(:,1), closed1(:,2), closed1(:,3));
    closed_r=sub2ind([row,col,ceng],closed2(:,1), closed2(:,2), closed2(:,3));
    % ѭ����������
    if max([opened_f;closed_f]==closed_r(end))
        end_point=closed_r(end);
        break;
    end
    if max([opened_r;closed_r]==closed_f(end))
        end_point=closed_f(end);
        break;
    end
end
T_all=toc; % ��ʱ����
             set(a,'handlevisibility','on');
             set(b,'handlevisibility','on');
%% ���ݺϲ�·��
[e_p_2d(1),e_p_2d(2),e_p_2d(3)]=ind2sub([row,col,ceng],end_point);
route1 =[e_p_2d];
route2 =[e_p_2d];
t=parent1(end_point);
while parent1(t)~=0
    [e_p_2d(1),e_p_2d(2),e_p_2d(3)]=ind2sub([row,col,ceng],t);
    route1=[route1;e_p_2d];
    t=parent1(t);
end
route1 = [route1;A];
route1=flipud(route1);
t=parent2(end_point);
while parent2(t)~=0
    [e_p_2d(1),e_p_2d(2),e_p_2d(3)]=ind2sub([row,col,ceng],t);
    route2=[route2;e_p_2d];
    t=parent2(t);
end
route2 = [route2;B];
route=[route1;route2(2:end,:)];
%% ���

Astar_route=(route-0.5)*delta;
Astar_route(:,1:2)=fliplr(Astar_route(:,1:2));
plot3(Astar_route(:,1),Astar_route(:,2),Astar_route(:,3),'b','LineWidth',2)
xlim([0 500]);
ylim([0 500]);
legend('Source','Goal','Obstacles','closeList_1','closeList_2','Path')
disp(['��������ʱ�䣺',num2str(T_all)]);
% view(-37.5,30)
view(-41,45)
for i=2:size(Astar_route(:,1))
    Q1=Astar_route(i-1,:);
    Q2=Astar_route(i,:);
dis(i)=((Q1(1)-Q2(1)).^2+(Q1(2)-Q2(2)).^2+(Q1(3)-Q2(3)).^2).^0.5;
end
disp(['Path length:',num2str(sum(dis,'all'))]);
disp(' Path as follows: (from S to E)');
disp(Astar_route);



%% ȥ������ڵ�
u=1;
o=0;
uuu=[]; 
Astar_route_simpify=[S];
while o < size(Astar_route,1) 
  for uu=u:size(Astar_route,1)
    if checkPath(Astar_route(u,1:3),Astar_route(uu,1:3), shp)
        uuu=[uuu;uu]; %�����洢���п��Ե���ĵ�
    end 
  end
     o=uuu(end);%uuu�е����һ���ڵ�
     Astar_route_simpify = [Astar_route_simpify;Astar_route(o,:)];
    u=o;
end


figure(2)
axis([0 col*delta 0 row*delta 0 ceng*delta])
hold on
plot3(S(1),S(2),S(3), 'o','markersize', 8,'markerfacecolor','g','MarkerEdgeColor', 'k')
plot3(E(1),E(2),E(3), 'o','markersize', 8,'markerfacecolor','r','MarkerEdgeColor', 'k')
shp = alphaShape(ob_coo(:,2),ob_coo(:,1),ob_coo(:,3));
plot(shp,'EdgeColor','k','FaceColor','k','FaceAlpha',0.5);
set(gcf,'position',[200,200,600,600]);
plot3(Astar_route(:,1),Astar_route(:,2),Astar_route(:,3),'b--','LineWidth',2);
plot3(Astar_route_simpify(:,1),Astar_route_simpify(:,2),Astar_route_simpify(:,3),'k','LineWidth',2); 
a=plot3(Astar_route_simpify(:,1),Astar_route_simpify(:,2),Astar_route_simpify(:,3),'o','markersize', 5,'markerfacecolor',[0 .7 .7],'MarkerEdgeColor',[0 .7 .7]); 
if Astar_route_simpify(1,1) ~= S(1,1)
set(a,'handlevisibility','on');
end
legend('Source','Goal','Obstacles','Astar-path','Astar-path simpify','Key point')
xlim([0 500]);
ylim([0 500]);
% view(0,90)
% view(-37.5,30)
view(-41,45)
for j=2:size(Astar_route_simpify(:,1))
    Q3=Astar_route_simpify(j-1,:);
    Q4=Astar_route_simpify(j,:);
dis2(j)=((Q3(1)-Q4(1)).^2+(Q3(2)-Q4(2)).^2+(Q3(3)-Q4(3)).^2).^0.5;
end
disp(['Bidirectional Astar-Path-Simpify length:',num2str(sum(dis2,'all'))]);
disp('Bidirectional Astar-Path-Simpify as follows: (from S to E)');
disp(Astar_route_simpify);
set(gca,'FontSize',16,'Fontname', 'Times New Roman');
save Global_path Astar_route Astar_route_simpify
