function [b,dis]=bpm3_fun(p,n2)
%n2=100;
%p=[0 50;50 200;150 150;400 250];
n=size(p,1);
t=linspace(0,1,n2)';   %����һ����β�ֱ�Ϊ0��1��100����������0,1������������
b=0;
for k=0:n-1
    Bni=nchoosek(n-1,k)*t.^k.*(1-t).^(n-1-k);
    b=b+Bni*p(k+1,:);
end
dis=0;
for i=2:size(b,1)
    dis=dis+norm(b(i,:)-b(i-1,:));
end