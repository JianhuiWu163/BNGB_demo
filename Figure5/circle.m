%******************* 画圆函数******************%
% (cx,cy) 为圆心坐标
% R为半径
% nb_pts为圆周上的点数
%*********************************************%
function [x,y] = circle(cx,cy,R,nb_pts)    
alpha = 0:pi/nb_pts:2*pi;
x = cx + R*cos(alpha); 
y = cy + R*sin(alpha); 