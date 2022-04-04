%**********************************************************%
% ��������˻���ʼλ�����ɺ���
%%�������
% x,y,theta:��������˻�x���ꡢy����ͺ���
%%�������
% x0,y0��Բ�����ꣻd:��ȫ���ࣻNum������
%**********************************************************%
function [x,y,theta]=Scouter_PosInitial(x0,y0,d,Num)
i = 1;
Flag = 0;
while(1)
    r = 40 * rand;       	% ������ɰ뾶
    theta(i) = 2 * pi * rand;	% ������ɽǶ�
    % �õ��������---�����ɢ����
    x(i) = x0 + r * cos(theta(i));  
    y(i) = y0 + r * sin(theta(i));
	
	if(i>1)
		for j=1:i-1
            dij = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2);
			if(dij<=d) 
				Flag = 1;
			end
        end
    end
    %--------------------%
    if(Flag==0)
        if(i>=Num)
            return;
        else
            i = i + 1;
        end
    end
    Flag = 0;
end  

