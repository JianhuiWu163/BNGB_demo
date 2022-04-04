%**********************************************************%
% CircleRandPoints��Բ����������ɺ���
% x0,y0��Բ�ĺ������ꣻR���뾶��num_Dian���������
%**********************************************************%
function [x,y]=CircleRandPoints(x0,y0,R,Rmax,num_Dian)
i = 1;
while(1)
    r = R;                  % ������ɰ뾶
    seta = 2 * pi * rand;	% ������ɽǶ�
    % �õ��������---�����ɢ����
    x(i) = x0 + r * cos(seta);  
    y(i) = y0 + r * sin(seta);
    
    % ���б߽�����
	if(sqrt(x(i)^2+y(i)^2) <= Rmax)
        if(i>=num_Dian)
            return;
        else
            i = i + 1;
        end
    end
end    

