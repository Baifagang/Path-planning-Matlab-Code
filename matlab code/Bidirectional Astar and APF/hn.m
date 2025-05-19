function v = hn(point,endp)
% %HN    计算该点到终点的曼哈顿距离
v = abs(point(1)-endp(1)) + abs(point(2)-endp(2))+ abs(point(3)-endp(3));
% %HN    计算该点到终点的欧式距离
% v = sqrt((point(1)-endp(1))^2 + (point(2)-endp(2))^2+ (point(3)-endp(3))^2);

% % %HN    计算该点到终点的切比雪夫距离
% h_diagonal=min(abs(point(1)-endp(1)),abs(point(2)-endp(2)));
% h_straight=abs(point(1)-endp(1))+abs(point(2)-endp(2));
% v=sqrt(2)*h_diagonal+(h_straight-2*h_diagonal);
end