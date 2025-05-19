function v = hn(p1,p2)
%HN    计算该点到终点的曼哈顿距离

%v = abs(point(1)-endp(1)) + abs(point(2)-endp(2));

%v=0.2*(sum(abs(p1(1:2)-p2(1:2)))*106+abs(p1(3)-p2(3))*340)*2.25*delta;

v=norm(p1-p2)*80;
end