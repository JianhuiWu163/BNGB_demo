%**********************************************************%
% 侦察者无人机初始位置生成函数
%%输出变量
% x,y,theta:侦察者无人机x坐标、y坐标和航向
%%输入变量
% x0,y0：圆心坐标；d:安全航距；Num：数量
%**********************************************************%
function [x,y,theta]=Scouter_PosInitial(x0,y0,d,Num)
i = 1;
Flag = 0;
while(1)
    r = 40 * rand;       	% 随机生成半径
    theta(i) = 2 * pi * rand;	% 随机生成角度
    % 得到点的坐标---随机扩散方程
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

