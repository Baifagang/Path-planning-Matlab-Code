function v = gn_bid(point,G,parent)
[row,col,ceng]=size(G);
pr = parent(point(1),point(2),point(3)); %һά����
          
%һά����ת��Ϊ��ά����
[pc(1),pc(2),pc(3)] = ind2sub([row,col,ceng],pr);    
ed = norm(pc-point);   % ŷʽ����
v = G(pr) + ed;
end



