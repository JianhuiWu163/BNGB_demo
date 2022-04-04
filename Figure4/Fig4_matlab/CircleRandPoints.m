%**********************************************************%
% CircleRandPoints：圆内随机点生成函数
% x0,y0：圆心横纵坐标；R：半径；num_Dian：点的数量
%**********************************************************%
function [x,y]=CircleRandPoints(x0,y0,R,Rmax,num_Dian)
i = 1;
while(1)
    r = R;                  % 随机生成半径
    seta = 2 * pi * rand;	% 随机生成角度
    % 得到点的坐标---随机扩散方程
    x(i) = x0 + r * cos(seta);  
    y(i) = y0 + r * sin(seta);
    
    % 飞行边界限制
	if(sqrt(x(i)^2+y(i)^2) <= Rmax)
        if(i>=num_Dian)
            return;
        else
            i = i + 1;
        end
    end
end    

