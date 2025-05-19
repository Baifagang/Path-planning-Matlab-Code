function v = gn_bid(point,G,parent)
[row,col,ceng]=size(G);
pr = parent(point(1),point(2),point(3)); %一维索引
          
%一维索引转换为二维索引
[pc(1),pc(2),pc(3)] = ind2sub([row,col,ceng],pr);    
ed = norm(pc-point);   % 欧式距离
v = G(pr) + ed;
end



