%*********************************************************************%
% AnnulusRandPoints��Բ������������ɺ���
% x0,y0��Բ�ĺ������ꣻRmin,Rmax��Բ���ھ����⾶��num_Dian���������
%*********************************************************************%
function [x,y,seta]=AnnulusRandPoints(x0,y0,Rmin,Rmax,num_Dian)
% �������num_Dian���뾶 
r = Rmin + rand(1,num_Dian)*(Rmax-Rmin); 
% �õ����ɵ�ĽǶȣ������ü�������ʽ������ 
seta = 2*pi*rand(1,length(r)); 
% �õ�������� 
x = x0 + r.*cos(seta); 
y = y0 + r.*sin(seta);

