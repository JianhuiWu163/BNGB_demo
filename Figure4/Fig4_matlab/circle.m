%******************* ��Բ����******************%
% (cx,cy) ΪԲ������
% RΪ�뾶
% nb_ptsΪԲ���ϵĵ���
%*********************************************%
function [x,y] = circle(cx,cy,R,nb_pts)    
alpha = 0:pi/nb_pts:2*pi;
x = cx + R*cos(alpha); 
y = cy + R*sin(alpha); 