function [yaw_p,min_dis]=yaw_fun(point,route)
route_temp=route(1,:);
for i=2:size(route,1)
    Q1=route(i-1,:);
    Q2=route(i,:);
    n=ceil(norm(Q1-Q2));
    for j=1:n
        temp=Q1+(Q2-Q1)*j/n;
        route_temp=[route_temp;temp];
    end
end
dis=((point(1)-route_temp(:,1)).^2+(point(2)-route_temp(:,2)).^2+(point(3)-route_temp(:,3)).^2).^0.5;
[min_dis,ind]=min(dis);
yaw_p=route_temp(ind,:);
end