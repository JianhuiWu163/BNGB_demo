%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ����������
% ��n>=101ʱ�����ַ�����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear all;close all;
init_angle = [2.20983467029187,2.20368644354434];
vmax = 40;                    	% ��������ٶ�m/s
vi = vmax;                    	% �����ٶ�m/s
delta_t = 0.01;              	% ����ͻ��������Ӧʱ��s
ScouterPos.x = zeros(1,2);     	% ��������˻���x����
ScouterPos.y = zeros(1,2);    	% ��������˻���y����
ScouterPos.angle = zeros(1,2);	% ��������˻��ĺ���
Last_ScouterPos = ScouterPos;

Last_ScouterPos.x(1) = 50 + 40 * cos(pi+init_angle(1));
Last_ScouterPos.y(1) = 50 + 40 * sin(pi+init_angle(1));
Last_ScouterPos.x(2) = 50 + 41.8 * cos(pi+init_angle(2));
Last_ScouterPos.y(2) = 50 + 41.8 * sin(pi+init_angle(2));

for n=1:10000
    for i=1:2
        ScouterPos.x(i) = Last_ScouterPos.x(i) + vi * delta_t * cos(ScouterPos.angle(i));  
        ScouterPos.y(i) = Last_ScouterPos.y(i) + vi * delta_t * sin(ScouterPos.angle(i));
    end
    plot(ScouterPos.x(1),ScouterPos.y(1),'.r',ScouterPos.x(2),ScouterPos.y(2),'.b');
    hold on;
    axis([-100 100 -100 100])
    
    Last_ScouterPos = ScouterPos;
    Dij = sqrt((ScouterPos.x(1)-ScouterPos.x(2))^2 + (ScouterPos.y(1)-ScouterPos.y(2))^2);
    if(Dij <= 1.8)
        ScouterPos.angle(1) = Last_ScouterPos.angle(1) + pi;
        ScouterPos.angle(2) = Last_ScouterPos.angle(2) + pi;
    else
        ScouterPos.angle(1) = init_angle(1);
        ScouterPos.angle(2) = init_angle(2);
    end
    pause(0.1);
    hold off;
end