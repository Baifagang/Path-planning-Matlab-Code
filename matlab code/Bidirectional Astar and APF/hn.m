function v = hn(point,endp)
% %HN    ����õ㵽�յ�������پ���
v = abs(point(1)-endp(1)) + abs(point(2)-endp(2))+ abs(point(3)-endp(3));
% %HN    ����õ㵽�յ��ŷʽ����
% v = sqrt((point(1)-endp(1))^2 + (point(2)-endp(2))^2+ (point(3)-endp(3))^2);

% % %HN    ����õ㵽�յ���б�ѩ�����
% h_diagonal=min(abs(point(1)-endp(1)),abs(point(2)-endp(2)));
% h_straight=abs(point(1)-endp(1))+abs(point(2)-endp(2));
% v=sqrt(2)*h_diagonal+(h_straight-2*h_diagonal);
end