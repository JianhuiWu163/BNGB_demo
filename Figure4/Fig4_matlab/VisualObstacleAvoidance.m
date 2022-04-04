function Res = VisualObstacleAvoidance(ObsPos,UavPos,LastUavPos,TarPos,DetRadius,RePhase)
[r1,r2,ObsNum] = size(ObsPos);
Dis = sqrt((TarPos(1,1)-UavPos(1,1))^2 + (TarPos(2,1)-UavPos(2,1))^2);
% ���˻���Ŀ���Ƕ�
if(Dis>0)
    if((TarPos(2,1)-UavPos(2,1))>=0)
        TargetAngle = acos((TarPos(1,1)-UavPos(1,1))/Dis);
    else
    	TargetAngle = 2*pi - acos((TarPos(1,1)-UavPos(1,1))/Dis);
   	end
else
	TargetAngle = 0;
end
% Ŀ��㵽���˻��Ƕ�
UavAngle = TargetAngle + pi;

m = 0;
for n=1:ObsNum
    Dis = sqrt((UavPos(1,1)-ObsPos(1,1,n))^2 + (UavPos(2,1)-ObsPos(2,1,n))^2);
    if(Dis <= DetRadius)
        m = m + 1;
        if(Dis>0)
            if((ObsPos(2,1,n)-UavPos(2,1))>=0)
                ObsAngle(m) = acos((ObsPos(1,1,n)-UavPos(1,1))/Dis);
            else
                ObsAngle(m) = 2*pi - acos((ObsPos(1,1,n)-UavPos(1,1))/Dis);
            end
        else
            ObsAngle(m) = 0;
        end
        High(m) = Dis * sin(abs(ObsAngle(m)-TargetAngle));
    end
end

if(m>0)
    Res = max(ObsAngle)  + pi/180 + rand * pi/6;
%     if(min(High)<=SafeDis) 
%         Res = max(ObsAngle)  + pi/180 + rand * pi/4;
%     else
%         Res = TargetAngle + rand * pi/4;
%     end
else
	%���Լ�1����һʱ�̵������뵱ǰ����Ƚ��Ƿ���DetRadius��Χ�ڣ�����Res = max(ObsAngle)  + pi/180 + rand * pi/4;
    Dis = sqrt((UavPos(1,1)-LastUavPos(1,1))^2 + (UavPos(2,1)-LastUavPos(2,1))^2);
    if(Dis <= DetRadius)
        Res = RePhase  + pi/6 + rand * pi/6;
    else
        Res = TargetAngle + rand * pi/18;
%         Res = TargetAngle + rand * pi/4;    %����rand*pi/180��ֹ������
    end
end
