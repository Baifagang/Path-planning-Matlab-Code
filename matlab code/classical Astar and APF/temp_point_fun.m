function temp=temp_point_fun(p1,p2,smart_dis)
l=norm(p1-p2);
temp=[0 0 0];
temp(3)=p2(3)-(p2(3)-p1(3))*smart_dis/l;
temp(2)=p2(2)-(p2(2)-p1(2))*smart_dis/l;
temp(1)=p2(1)-(p2(1)-p1(1))*smart_dis/l;


