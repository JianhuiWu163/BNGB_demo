%*********************************************************************%
% AnnulusRandPoints：圆环内随机点生成函数
% x0,y0：圆心横纵坐标；Rmin,Rmax：圆环内径、外径；num_Dian：点的数量
%*********************************************************************%
function [x,y,seta]=AnnulusRandPoints(x0,y0,Rmin,Rmax,num_Dian)
% 随机生成num_Dian个半径 
r = Rmin + rand(1,num_Dian)*(Rmax-Rmin); 
% 得到生成点的角度，并利用极坐标形式画出点 
seta = 2*pi*rand(1,length(r)); 
% 得到点的坐标 
x = x0 + r.*cos(seta); 
y = y0 + r.*sin(seta);

