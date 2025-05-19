clc;clear;close all;

data1=xlsread('三维地图3.xlsx');
data1=data1(2:end,2:end);
[row,col]=size(data1);
ceng=25;
sign =zeros(row,col,ceng);
for i=1:row
    for j=1:col
        sign(i,j,1:data1(i,j)) = 1;
    end
end

% A=[10,13,2];                           %起点行列高值 [行,列,高]
A=[5,3,2];                               %起点行列高值 [行,列,高]
B=[85,93,8];                              %终点行列高值 [行,列,高]


% B=[2,35,24]; 
% A=[92,5,8];

delta=5;                                 %栅格边长
S=(A-0.5)*delta;
S(1:2)=fliplr(S(1:2));                   %起点坐标 
E=(B-0.5)*delta;
E(1:2)=fliplr(E(1:2));                   %终点坐标

ob_coo=[];                               %障碍物 [行,列,高]*delta
for i=1:row
    for j=1:col
        for k=1:ceng
            if sign(i,j,k)==1
               ob_coo = [ ob_coo;[i-0.5 j-0.5 k-0.5]*delta ];
            end
        end
    end
end

            % 考虑障碍物的影响范围
            mov = [1 0 0;1 0 1;1 0 -1;1 1 0;1 1 1;1 1 -1;1 -1 0;1 -1 1;1 -1 -1;
                   0 0 1;0 0 -1;0 1 0;0 1 1;0 1 -1;0 -1 0;0 -1 1;0 -1 -1;
                  -1 0 0;-1 0 1;-1 0 -1;-1 1 0;-1 1 1;-1 1 -1;-1 -1 0;-1 -1 1;-1 -1 -1]*delta;
            ob_influ=zeros(length(ob_coo(:,1))*size(mov,1),3);
            for io=1:length(ob_coo(:,1))
                for im=1:size(mov,1)
                    ob_influ(io*im,:) = ob_coo(io,:) + mov(im,:);
                end
            end
            ob_influ=unique(ob_influ(:,1:3),'rows');                       %障碍物影响 [行,列,高]*delta
            ob_influ2 = [ob_influ(:,2), ob_influ(:,1),ob_influ(:,3)];

            ob_influ2(ob_influ2(:,1)<=0,:)=[];ob_influ2(ob_influ2(:,1)>=row*delta,:)=[];
            ob_influ2(ob_influ2(:,2)<=0,:)=[];ob_influ2(ob_influ2(:,2)>=col*delta,:)=[];
            ob_influ2(ob_influ2(:,3)<=0,:)=[];ob_influ2(ob_influ2(:,3)>=ceng*delta,:)=[];

            ob_influ2_pos = [ob_influ2(:,2)/delta+0.5,ob_influ2(:,1)/delta+0.5,ob_influ2(:,3)/delta+0.5];
            
            for ii = 1: length(ob_influ2_pos(:,1))
                sign(ob_influ2_pos(ii,1),ob_influ2_pos(ii,2),ob_influ2_pos(ii,3))=1;
            end
ob_r=delta/2^0.5;
ob_num=size(ob_coo,1);

figure(1)
hold on
axis([0 col*delta 0 row*delta 0 ceng*delta])
plot3(S(1),S(2),S(3), 'o','markersize', 8,'markerfacecolor','g','MarkerEdgeColor', 'k')
plot3(E(1),E(2),E(3), 'o','markersize', 8,'markerfacecolor','r','MarkerEdgeColor', 'k')
shp = alphaShape(ob_coo(:,2),ob_coo(:,1),ob_coo(:,3));
plot(shp,'EdgeColor','k','FaceColor','k','FaceAlpha',0.6);
legend('Source','Goal','Obstacles')
set(gca,'FontSize',12,'Fontname', 'Times New Roman');
axis equal
% view(-37.5,30)
view(-41,45)
figure(2)
hold on
axis([0 col*delta 0 row*delta 0 ceng*delta])
plot3(S(1),S(2),S(3), 'o','markersize', 8,'markerfacecolor','g','MarkerEdgeColor', 'k');
plot3(E(1),E(2),E(3), 'o','markersize', 8,'markerfacecolor','r','MarkerEdgeColor', 'k');
plot3(ob_coo(:,2),ob_coo(:,1),ob_coo(:,3),'.');
set(gca,'FontSize',12,'Fontname', 'Times New Roman');
axis equal

figure(3)
hold on
plot3(ob_influ2_pos(:,2),ob_influ2_pos(:,1),ob_influ2_pos(:,3),'.');
set(gca,'FontSize',12,'Fontname', 'Times New Roman');
axis equal
save Environment_data data1 row col ceng sign A B S E ob_coo ob_influ2 delta ob_r ob_num
