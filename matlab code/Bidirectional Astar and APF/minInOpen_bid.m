function [index,mc] = minInOpen_bid(open,F)
%MININOPEN    ����open�б���fn��С�Ľڵ�
%   OPENΪ�����б�
%   

mv =  99999999; 
for ii=1:size(open,1)
     v = F(open(ii,1),open(ii,2),open(ii,3));
    if v<mv
        mv = v;
        mc = open(ii,:);
        index = ii;
    end
end
end
