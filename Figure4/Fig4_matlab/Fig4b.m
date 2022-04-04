clc;close all;clear all;
%*************************************************************************%
xMin = -200; xMax = 200; 
yMin = -200; yMax = 200;
outfile1 = 'Iters_UAV001.xlsx';
outfile2 = 'Dij_UAV001.xlsx';
DataBuff1 = zeros(1000,100);
% DataBuff2 = zeros(1000,1);

UAV_TotalNum = 2;	% ���˻�����

% TestNum = 1;        % ���Լ���
CollisionNum = 0;   % ���ϼ���
DijBefore = inf;

mu = ones(UAV_TotalNum,UAV_TotalNum);       % �����˻����ͻ״̬��1��ȫ��0С�ڰ�ȫ����
vmax = 20;                                  % ��������ٶ�m/s
vi = vmax;                                  % �����ٶ�m/s
Tmax = 1;                                   % ������λ��Ӧʱ��s
delta_t = 0.01;                             % ����ͻ��������Ӧʱ��s
mdelT = floor(Tmax/delta_t);
delta_Ri = vi * delta_t;                    % delta_t���ں��о���
dmin = 1;                                   % ��С���а�ȫ����
d = 2*vmax*delta_t + dmin;                  % ��ȫ����
DetectRadius = 20;                          % ̽��뾶

% �������Ŀ���
TargetNum = 1;                              % Ŀ������
TargetPos = zeros(2,TargetNum);           % Ŀ��x���ꡢy����

% ̽��Ŀ��
DetectTargetPos = zeros(2,TargetNum);     % �洢̽�⵽��Ŀ������
SimilarFlag = 0;
TargetCount = 0;

Dis = zeros(1,UAV_TotalNum);
last_Dis = 0;
ScouterPos = zeros(3,UAV_TotalNum);     	% ��������˻���x���ꡢy���ꡢ����
%%% [ScouterPos(1,:),ScouterPos(2,:),ScouterPos(3,:)] = Scouter_PosInitial(0,0,d,UAV_TotalNum);	% ���������������
% ScouterPos(1,1)=-2;
% ScouterPos(2,1)=0;
% ScouterPos(1,2)=2;
% ScouterPos(2,2)=0;
% TargetPos(1,1) = 0.85 * xMax * cos(1*pi/4); 
% TargetPos(2,1)= 0.85 * xMax * sin(1*pi/4);

ScouterPos(1,1)=2;
ScouterPos(2,1)=0;
ScouterPos(1,2)=-2;
ScouterPos(2,2)=0;
TargetPos(1,1) = 0.85 * xMax * cos(5*pi/4); 
TargetPos(2,1)= 0.85 * xMax * sin(5*pi/4);
% ���˻���Ŀ���Ƕ�
for i=1:UAV_TotalNum
    Dis(1,i) = sqrt((TargetPos(1,1)-ScouterPos(1,i))^2 + (TargetPos(2,1)-ScouterPos(2,i))^2);
    if(Dis(1,i)>0)
        if((TargetPos(2,1)-ScouterPos(2,i))>=0)
            ScouterPos(3,i) = acos((TargetPos(1,1)-ScouterPos(1,i))/Dis(1,i));
        else
            ScouterPos(3,i) = 2*pi - acos((TargetPos(1,1)-ScouterPos(1,i))/Dis(1,i));
        end
    else
        ScouterPos(3,i) = 0;
    end
end
Last_ScouterPos = ScouterPos;
%******************************************************************************%
figure(1)
set(gcf,'Position',[50 50 180 180]);    % ͼƬ��С 
set(gca,'Position',[.15 .12 .8 .8]);    % ��������ռ����
Dimensional2D(xMin,xMax,5);            	% ����ά����
axis([xMin xMax yMin yMax]);
set(gca,'Fontsize',8);                 	% ����̶������С
hold on;
plot(TargetPos(1,:),TargetPos(2,:),'sb');

% ����Ŀ�긽����̬�ϰ���
j = 0;
for i=1:TargetNum  
    Radius = sqrt(TargetPos(1,i)^2 + TargetPos(2,i)^2);
	if(-TargetPos(2,i)>=0) 
        Centerangle(1,i) = acos(-TargetPos(1,i)/Radius);
    else
        Centerangle(1,i) = 2*pi - acos(-TargetPos(1,i)/Radius);
    end 
    minAngle = Centerangle(1,i) - pi*70/180;
    maxAngle = Centerangle(1,i) + pi*70/180;
    j = 0;
    for phi = minAngle:pi/360:maxAngle
        j = j + 1;
        StaticObstaclePos(1,i,j) = TargetPos(1,i) + 50 * cos(phi);
        StaticObstaclePos(2,i,j) = TargetPos(2,i) + 50 * sin(phi); 
        Xj(i,j) = StaticObstaclePos(1,i,j);
        Yj(i,j) = StaticObstaclePos(2,i,j);
    end
    startj(i) = j + 1;
    plot(Xj(i,1:startj(i)-1),Yj(i,1:startj(i)-1),'.k','MarkerSize',3);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ObsType = 2;    % 0��͹��Բ����1������Բ����2��ֱ��
if(ObsType==0)
    for i=1:TargetNum
        minAngle = Centerangle(1,1) - pi*60/180;
        maxAngle = Centerangle(1,1) + pi*60/180;
        j = startj(1) - 1;
        for phi = minAngle:pi/1000:maxAngle
            j = j + 1;
            StaticObstaclePos(1,i,j) = TargetPos(1,i) + 110 * cos(phi); 
            StaticObstaclePos(2,i,j) = TargetPos(2,i) + 110 * sin(phi); 
            Xj(i,j) = StaticObstaclePos(1,i,j);
            Yj(i,j) = StaticObstaclePos(2,i,j);
        end
        endj(i) = j;
    end
elseif(ObsType==1)
    for i=1:TargetNum
        minAngle = Centerangle(1,1) + pi - pi*160/180;
        maxAngle = Centerangle(1,1) + pi + pi*160/180;
        j = startj(1) - 1;
        for phi = minAngle:pi/1000:maxAngle
            j = j + 1;
            StaticObstaclePos(1,i,j) = 110 * cos(phi);
            StaticObstaclePos(2,i,j) = 110 * sin(phi); 
            Xj(i,j) = StaticObstaclePos(1,i,j);
            Yj(i,j) = StaticObstaclePos(2,i,j);
        end
        endj(i) = j;
    end
elseif(ObsType==2)
    for i=1:TargetNum
        minAngle = mod(Centerangle(1,1) + pi + 90*pi/180,2*pi);
        Xtemp = 80 * cos(Centerangle(1,1) + pi);
        Ytemp = 80 * sin(Centerangle(1,1) + pi); 
        j = startj(1) - 1;
        for zhinum = 500:-1:1 
            j = j + 1;
            StaticObstaclePos(1,i,j) = Xtemp + zhinum*cos(minAngle) * 0.2; 
            StaticObstaclePos(2,i,j) = Ytemp + zhinum*sin(minAngle) * 0.2; 
            Xj(i,j) = StaticObstaclePos(1,i,j);
            Yj(i,j) = StaticObstaclePos(2,i,j);
        end
        j = j + 1;
        StaticObstaclePos(1,i,j) = Xtemp; 
        StaticObstaclePos(2,i,j) = Ytemp; 
        Xj(i,j) = StaticObstaclePos(1,i,j);
        Yj(i,j) = StaticObstaclePos(2,i,j);

        for zhinum = 1:500
            j = j + 1;
            StaticObstaclePos(1,i,j) = Xtemp -  zhinum*cos(minAngle) * 0.2; 
            StaticObstaclePos(2,i,j) = Ytemp -  zhinum*sin(minAngle) * 0.2; 
            Xj(i,j) = StaticObstaclePos(1,i,j);
            Yj(i,j) = StaticObstaclePos(2,i,j);
        end
        endj(i) = j;
    end
end
plot(Xj(1,startj(i):endj(i)),Yj(1,startj(i):endj(i)),'.k','MarkerSize',3);
%%==��U���ϰ���==%
if(ObsType==2)
    for i=1:TargetNum
        minAngle = mod(Centerangle(1,1) + pi + 0*pi/180,2*pi);
        Xtemp = StaticObstaclePos(1,i,startj(i));
        Ytemp = StaticObstaclePos(2,i,startj(i));
        for zhinum = 1:500
            j = j + 1;
            StaticObstaclePos(1,i,j) = Xtemp - zhinum*cos(minAngle) * 0.2; 
            StaticObstaclePos(2,i,j) = Ytemp - zhinum*sin(minAngle) * 0.2; 
            XL1(i,zhinum) = StaticObstaclePos(1,i,j);
            YL1(i,zhinum) = StaticObstaclePos(2,i,j);
        end
        minAngle = mod(Centerangle(1,1) + pi - 0*pi/180,2*pi);
        Xtemp = StaticObstaclePos(1,i,endj(i));
        Ytemp = StaticObstaclePos(2,i,endj(i));
        for zhinum = 1:500
            j = j + 1;
            StaticObstaclePos(1,i,j) = Xtemp -  zhinum*cos(minAngle) * 0.2; 
            StaticObstaclePos(2,i,j) = Ytemp -  zhinum*sin(minAngle) * 0.2; 
            XL2(i,zhinum) = StaticObstaclePos(1,i,j);
            YL2(i,zhinum) = StaticObstaclePos(2,i,j);
        end
    end
    plot(XL1,YL1,'.k','MarkerSize',3);
    plot(XL2,YL2,'.k','MarkerSize',3);
end
SONum = length(StaticObstaclePos(1,1,:));
smu = ones(UAV_TotalNum,TargetNum,SONum);	% ��̬�ϰ����ĳ�ͻ״̬��1��ȫ��0С�ڰ�ȫ����
ObsAngle = zeros(UAV_TotalNum,SONum);
ObsBuffer = zeros(2,UAV_TotalNum,SONum);
ObsBuferfNum = zeros(1,UAV_TotalNum); 
%*********************************************************************%
for i=1:UAV_TotalNum
 	if(i==1)
        plot(ScouterPos(1,i),ScouterPos(2,i),'r.','MarkerSize',2);
 	else
 		plot(ScouterPos(1,i),ScouterPos(2,i),'.','color',[0/255 153/255 51/255],'MarkerSize',2);
 	end
end

UavPos1(1,1) = ScouterPos(1,1);
UavPos1(2,1) = ScouterPos(2,1);
UavPos2(1,1) = ScouterPos(1,2);
UavPos2(2,1) = ScouterPos(2,2);

Theta_1s = zeros(1,UAV_TotalNum);
Xtemp = 0;
Ytemp = 0;

CollisionFlag = 0;    %����
BeforePos = zeros(3,UAV_TotalNum); 
ContinumFlag = zeros(1,UAV_TotalNum);
infFlag = zeros(1,UAV_TotalNum);
KeepFlag = zeros(1,UAV_TotalNum);
FinishFlag = zeros(1,UAV_TotalNum);
FlagState = zeros(1,UAV_TotalNum);
BiasAngle = 0;

% OffsetDire = ones(1,UAV_TotalNum);  % ������ñ��Ϸ���
% for i=1:UAV_TotalNum
%     if(rand < 0.5)
%         OffsetDire(1,i) = -1;
%     end
% end
OffsetDire(1,1) = 1;
OffsetDire(1,2) = -1;
%*************************************************************************%
for iters=1:30000000   
	if(sum(FinishFlag)>=UAV_TotalNum)
		break;
	end
	
	for i=1:UAV_TotalNum
		if(FinishFlag(1,i)~=1)
            if(mod(iters,mdelT)==0)
                BoundaryFlag = 1;
                EndlessLoop_num = 0;
                while(BoundaryFlag)
					last_Dis = Dis(1,i);
                    Dis(1,i) = sqrt((TargetPos(1,1)-ScouterPos(1,i))^2 + (TargetPos(2,1)-ScouterPos(2,i))^2);
                    if((prod(mu(i,:),2)==1)&&(prod3(smu(i,:,:))==1))
                        % ���˻���Ŀ���Ƕ�
                        if(Dis(1,i)>0)
                            if((TargetPos(2,1)-ScouterPos(2,i))>=0)
                                TargetAngle = acos((TargetPos(1,1)-ScouterPos(1,i))/Dis(1,i));
                            else
                                TargetAngle = 2*pi - acos((TargetPos(1,1)-ScouterPos(1,i))/Dis(1,i));
                            end
                        else
                            TargetAngle = 0;
                        end

                        StaticObstacleNumber = 0;
                        StaticObstacleFlag = 0;
                        for n=1:SONum
                            ObsAngle(i,n) = 0;
                        end
                        TarObsLineFlag = 0;
                        for n=1:SONum    
                            Dobs = sqrt((ScouterPos(1,i)-StaticObstaclePos(1,1,n))^2 + (ScouterPos(2,i)-StaticObstaclePos(2,1,n))^2);
                            if(Dobs <= DetectRadius)
                                StaticObstacleFlag = 1;
                                StaticObstacleNumber = StaticObstacleNumber + 1;
                                if(Dobs>0)
                                    if((StaticObstaclePos(2,1,n)-ScouterPos(2,i))>=0)
                                        ObsAngle(i,StaticObstacleNumber) = acos((StaticObstaclePos(1,1,n)-ScouterPos(1,i))/Dobs);
                                    else
                                        ObsAngle(i,StaticObstacleNumber) = 2*pi - acos((StaticObstaclePos(1,1,n)-ScouterPos(1,i))/Dobs);
                                    end
                                else
                                    ObsAngle(i,StaticObstacleNumber) = 0;
                                end
                                % ���ӷ�Χ�ڣ��ж��ϰ����Ƿ��谭���˻�����Ŀ���
                                if(((abs(TargetAngle - ObsAngle(i,StaticObstacleNumber))<= (5*pi/180))&&(Dobs<Dis(1,i)))||(Dis(1,i)>last_Dis))  % 5*pi/180�Ƿ�ֹ����ʱ�����õ��ϰ���㲻���ܼ�
                                    TarObsLineFlag = 1;			% ����TarObsLineFlag���ֵ�����
                                end
								% �洢��̽�⵽���ϰ���
								RepeatFlag = 0;
								if(ObsBuferfNum(i)>0)		% if(ObsBuferfNum(i)>1)
									for j=1:ObsBuferfNum(i) % for j=1:ObsBuferfNum(i)-1
										if((ObsBuffer(1,i,j)==StaticObstaclePos(1,1,n))&&(ObsBuffer(2,i,j)==StaticObstaclePos(2,1,n)))
											RepeatFlag = 1;		% �ж�����֪�ϰ���������ͬ�����洢 
											break;
										end
									end
								end
                                if(RepeatFlag==0)		% �޳���ͬ���ϵ����꣬�Դ˼��ٴ洢�ռ�
									ObsBuferfNum(i) = ObsBuferfNum(i) + 1;
									ObsBuffer(1,i,ObsBuferfNum(i)) = StaticObstaclePos(1,1,n);	% ��֪�ϰ���
									ObsBuffer(2,i,ObsBuferfNum(i)) = StaticObstaclePos(2,1,n);
								end
                            end
                        end
                        % �ж���֪�ϰ����Ƿ��赲���˻���Ŀ���(�������Բ���Ƕȴ���180��)
                        if((StaticObstacleFlag==1)&&(TarObsLineFlag~=1))
                            for n=1:ObsBuferfNum(i)    
                                Dobs = sqrt((ScouterPos(1,i)-ObsBuffer(1,i,n))^2 + (ScouterPos(2,i)-ObsBuffer(2,i,n))^2);
                             	if(Dobs>0)
                                	if((ObsBuffer(2,i,n)-ScouterPos(2,i))>=0)
                                        Temp = acos((ObsBuffer(1,i,n)-ScouterPos(1,i))/Dobs);
                                    else
                                    	Temp = 2*pi - acos((ObsBuffer(1,i,n)-ScouterPos(1,i))/Dobs);
                                    end
                                else
                                	Temp = 0;
                                end
                              	% ���ӷ�Χ�ڣ��ж��ϰ����Ƿ��谭���˻�����Ŀ���
                            	if((abs(TargetAngle - Temp)<= (5*pi/180))&&(Dobs<Dis(1,i)))  	% 5*pi/180�Ƿ�ֹ����ʱ�����õ��ϰ���㲻���ܼ�
                                	TarObsLineFlag = 1;                                     % ����TarObsLineFlag���ֵ�����
									break;
                                end
                        	end
                        end                        
                        %=====================================================================%
                        if((StaticObstacleFlag==1)&&(TarObsLineFlag==1))
							% ������任--y��ָ��Ŀ���
							if(TargetPos(2,1)>=0)
								BiasAngle = acos(TargetPos(1,1)/sqrt(TargetPos(1,1)^2+TargetPos(2,1)^2));
							else
								BiasAngle = 2*pi - acos(TargetPos(1,1)/sqrt(TargetPos(1,1)^2+TargetPos(2,1)^2));
                            end
							for j=1:StaticObstacleNumber
% 								ObsAngle(i,j) = mod(ObsAngle(i,j)-(BiasAngle - pi/2),2*pi);
                                ObsAngle(i,j) = mod(ObsAngle(i,j)-BiasAngle,2*pi);
								if(ObsAngle(i,j)<0)
									ObsAngle(i,j) = ObsAngle(i,j) + 2*pi;
								end
							end
                            % ð������--��С��������
                        	for n=1:StaticObstacleNumber
                                for j=1:StaticObstacleNumber-n
                                	if(ObsAngle(i,j)>ObsAngle(i,j+1))
                                    	Temp = ObsAngle(i,j+1);
                                     	ObsAngle(i,j+1) = ObsAngle(i,j);
                                      	ObsAngle(i,j) = Temp;
                                    end
                                end
                            end
                            % �������˻�����
                            if(OffsetDire(1,i)==1)  % ��ʱ��
                            	for j=1:StaticObstacleNumber-1
                                 	if(abs(ObsAngle(i,j) - ObsAngle(i,j+1))>(20*pi/180))    % ���Ƕ��Ƿ�����
%                                     	MaxObsAngle = ObsAngle(i,j)+(BiasAngle - pi/2);
                                        MaxObsAngle = ObsAngle(i,j)+BiasAngle;
                                        break;
                                    end
                                    if(j==(StaticObstacleNumber-1))
%                                     	MaxObsAngle = ObsAngle(i,StaticObstacleNumber)+(BiasAngle - pi/2);
                                        MaxObsAngle = ObsAngle(i,StaticObstacleNumber)+BiasAngle;
                                    end
                                end
                                ScouterPos(3,i) = MaxObsAngle + OffsetDire(1,i) * (pi/18 + rand*pi/6);
                            elseif(OffsetDire(1,i)==-1) %˳ʱ��
                                if((ObsAngle(i,StaticObstacleNumber)>350*pi/180)&&(ObsAngle(i,1)<10*pi/180))
                                    % ���Ƕ��Ƿ�����
                                    for j=StaticObstacleNumber:-1:2
                                        if(abs(ObsAngle(i,j) - ObsAngle(i,j-1))>(20*pi/180))    % 0�ȸ����ڽ�����λƫ��ϴ�����ѡ��20*pi/180
%                                             MinObsAngle = ObsAngle(i,j)+(BiasAngle - pi/2);
                                            MinObsAngle = ObsAngle(i,j)+BiasAngle;
                                            break;
                                        end
                                    end
                                else
%                                     MinObsAngle = ObsAngle(i,1)+(BiasAngle - pi/2);
                                    MinObsAngle = ObsAngle(i,1)+BiasAngle;
                                end
                                ScouterPos(3,i) = MinObsAngle + OffsetDire(1,i) * (pi/18 + rand*pi/6); 
							end
							KeepFlag(i) = 0;
                        else
                            if(Dis(1,i)>=10)
                                ScouterPos(3,i) = TargetAngle + OffsetDire(1,i) * rand * pi/4;    %����rand*pi/4��ֹ������
                            else
                            	ScouterPos(3,i) = TargetAngle;
                            	%%����Ŀ�ĵ���ͣ��������
                             	if(Dis(1,i)<0.8)
                                    FinishFlag(1,i) = 1;
                                end
                            end
                        end
                    else
                        ScouterPos(3,i)= Last_ScouterPos(3,i) + pi;
                        BoundaryFlag = 0;
                        CollisionFlag = CollisionFlag + 1;		% ����
                    end

                    % �߽�����(ֻ�м��1s���ܵõ�����)
                    Xtemp = Last_ScouterPos(1,i) + vi * Tmax * cos(ScouterPos(3,i));
                    Ytemp = Last_ScouterPos(2,i) + vi * Tmax * sin(ScouterPos(3,i));
                    if(sqrt(Xtemp^2 + Ytemp^2) <= xMax)
                        BoundaryFlag = 0;
                    else
                        if(EndlessLoop_num>=500)    % ��ֹ��ѭ��
                            BoundaryFlag = 0;
                            EndlessLoop_num = 0;
                            if(Last_ScouterPos(2,i)<=0)
                                ScouterPos(3,i) = acos(-Last_ScouterPos(1,i)/sqrt(Last_ScouterPos(1,i)^2+Last_ScouterPos(2,i)^2));
                            else
                                ScouterPos(3,i) = 2*pi - acos(-Last_ScouterPos(1,i)/sqrt(Last_ScouterPos(1,i)^2+Last_ScouterPos(2,i)^2));
                            end
                        else
                            EndlessLoop_num = EndlessLoop_num + 1;
                        end
                        Xtemp = Last_ScouterPos(1,i) + vi * Tmax * cos(ScouterPos(3,i));
                        Ytemp = Last_ScouterPos(2,i) + vi * Tmax * sin(ScouterPos(3,i));
                        Theta_1s(i) = ScouterPos(3,i);
                    end
                end
            else
                if((prod(mu(i,:),2)==1)&&(prod3(smu(i,:,:))==1))
                    ScouterPos(3,i) = Last_ScouterPos(3,i);
                    if(sqrt(Xtemp^2 + Ytemp^2) > xMax)
                        ScouterPos(3,i) = Theta_1s(i);
                    end
                else
                    ScouterPos(3,i) = Last_ScouterPos(3,i) + pi;
                end

                %%����Ŀ�ĵ���ͣ��������
                if(sqrt((TargetPos(1,1)-ScouterPos(1,i))^2 + (TargetPos(2,1)-ScouterPos(2,i))^2)<0.8)
                    FinishFlag(1,i) = 1;
                end
            end
            ScouterPos(3,i) = mod(ScouterPos(3,i),2*pi);
            ScouterPos(1,i) = Last_ScouterPos(1,i) + vi * delta_t * cos(ScouterPos(3,i));  
            ScouterPos(2,i) = Last_ScouterPos(2,i) + vi * delta_t * sin(ScouterPos(3,i));
%             plot(ScouterPos(1,i),ScouterPos(2,i),'r.','MarkerSize',2);
            if(i==1)
                plot(ScouterPos(1,i),ScouterPos(2,i),'r.','MarkerSize',2);
            else
                plot(ScouterPos(1,i),ScouterPos(2,i),'.','color',[0/255 153/255 51/255],'MarkerSize',2);
            end
            UavPos1(1,iters+1) = ScouterPos(1,1);
            UavPos1(2,iters+1) = ScouterPos(2,1);
            UavPos2(1,iters+1) = ScouterPos(1,2);
            UavPos2(2,iters+1) = ScouterPos(2,2);
            hold on;
		end
	end
    Last_ScouterPos = ScouterPos;
    if(mod(iters,mdelT)==0)
        BeforePos = ScouterPos;
    end
    %*********************************************************************%
    % ��������˻����ͻ״̬
    for i=1:UAV_TotalNum
        for j=1:UAV_TotalNum
			if((FinishFlag(1,i)==1)||(FinishFlag(1,j)==1))
				mu(i,j) = 1;
			else
				if(j==i)
					Dij = 0;
					mu(i,j) = 1;
				else
					Dij = sqrt((ScouterPos(1,i)-ScouterPos(1,j))^2 + (ScouterPos(2,i)-ScouterPos(2,j))^2);                    
					if(Dij > d)
						mu(i,j) = 1;
					else
						mu(i,j) = 0;
%                         CollisionNum = CollisionNum + 1;
%                         DataBuff2(CollisionNum) = Dij; 
					end
				end
			end
        end
        
		for j=1:TargetNum
			for	k=1:SONum	
				Dij = sqrt((ScouterPos(1,i)-StaticObstaclePos(1,j,k))^2 + (ScouterPos(2,i)-StaticObstaclePos(2,j,k))^2);
                if(Dij < DijBefore)
                    DijBefore = Dij;
                end
				if((Dij > d)||(FinishFlag(1,i)==1))
					smu(i,j,k) = 1;
				else
					smu(i,j,k) = 0;
%                     CollisionNum = CollisionNum + 1;
%                     DataBuff2(CollisionNum) = Dij; 
				end
			end
		end
    end
	pause(0.001); 
%     hold off;
end


figure(2)
set(gcf,'Position',[50 50 180 180]);    % ͼƬ��С 
set(gca,'Position',[.15 .12 .8 .8]);    % ��������ռ����
Dimensional2D(xMin,xMax,5);            	% ����ά����
axis([xMin xMax yMin yMax]);
set(gca,'Fontsize',8);                 	% ����̶������С
hold on;
plot(TargetPos(1,:),TargetPos(2,:),'sb');

% ����Ŀ�긽����̬�ϰ���
j = 0;
for i=1:TargetNum  
    Radius = sqrt(TargetPos(1,i)^2 + TargetPos(2,i)^2);
	if(-TargetPos(2,i)>=0) 
        Centerangle(1,i) = acos(-TargetPos(1,i)/Radius);
    else
        Centerangle(1,i) = 2*pi - acos(-TargetPos(1,i)/Radius);
    end 
    minAngle = Centerangle(1,i) - pi*70/180;
    maxAngle = Centerangle(1,i) + pi*70/180;
    j = 0;
    for phi = minAngle:pi/360:maxAngle
        j = j + 1;
        StaticObstaclePos(1,i,j) = TargetPos(1,i) + 50 * cos(phi);
        StaticObstaclePos(2,i,j) = TargetPos(2,i) + 50 * sin(phi); 
        Xj(i,j) = StaticObstaclePos(1,i,j);
        Yj(i,j) = StaticObstaclePos(2,i,j);
    end
    startj(i) = j + 1;
    plot(Xj(i,1:startj(i)-1),Yj(i,1:startj(i)-1),'.k','MarkerSize',3);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ObsType = 2;    % 0��͹��Բ����1������Բ����2��ֱ��
if(ObsType==0)
    for i=1:TargetNum
        minAngle = Centerangle(1,1) - pi*60/180;
        maxAngle = Centerangle(1,1) + pi*60/180;
        j = startj(1) - 1;
        for phi = minAngle:pi/1000:maxAngle
            j = j + 1;
            StaticObstaclePos(1,i,j) = TargetPos(1,i) + 110 * cos(phi); 
            StaticObstaclePos(2,i,j) = TargetPos(2,i) + 110 * sin(phi); 
            Xj(i,j) = StaticObstaclePos(1,i,j);
            Yj(i,j) = StaticObstaclePos(2,i,j);
        end
        endj(i) = j;
    end
elseif(ObsType==1)
    for i=1:TargetNum
        minAngle = Centerangle(1,1) + pi - pi*160/180;
        maxAngle = Centerangle(1,1) + pi + pi*160/180;
        j = startj(1) - 1;
        for phi = minAngle:pi/1000:maxAngle
            j = j + 1;
            StaticObstaclePos(1,i,j) = 110 * cos(phi);
            StaticObstaclePos(2,i,j) = 110 * sin(phi); 
            Xj(i,j) = StaticObstaclePos(1,i,j);
            Yj(i,j) = StaticObstaclePos(2,i,j);
        end
        endj(i) = j;
    end
elseif(ObsType==2)
    for i=1:TargetNum
        minAngle = mod(Centerangle(1,1) + pi + 90*pi/180,2*pi);
        Xtemp = 80 * cos(Centerangle(1,1) + pi);
        Ytemp = 80 * sin(Centerangle(1,1) + pi); 
        j = startj(1) - 1;
        for zhinum = 500:-1:1 
            j = j + 1;
            StaticObstaclePos(1,i,j) = Xtemp + zhinum*cos(minAngle) * 0.2; 
            StaticObstaclePos(2,i,j) = Ytemp + zhinum*sin(minAngle) * 0.2; 
            Xj(i,j) = StaticObstaclePos(1,i,j);
            Yj(i,j) = StaticObstaclePos(2,i,j);
        end
        j = j + 1;
        StaticObstaclePos(1,i,j) = Xtemp; 
        StaticObstaclePos(2,i,j) = Ytemp; 
        Xj(i,j) = StaticObstaclePos(1,i,j);
        Yj(i,j) = StaticObstaclePos(2,i,j);

        for zhinum = 1:500
            j = j + 1;
            StaticObstaclePos(1,i,j) = Xtemp -  zhinum*cos(minAngle) * 0.2; 
            StaticObstaclePos(2,i,j) = Ytemp -  zhinum*sin(minAngle) * 0.2; 
            Xj(i,j) = StaticObstaclePos(1,i,j);
            Yj(i,j) = StaticObstaclePos(2,i,j);
        end
        endj(i) = j;
    end
end
plot(Xj(1,startj(i):endj(i)),Yj(1,startj(i):endj(i)),'.k','MarkerSize',3);
%%==��U���ϰ���==%
if(ObsType==2)
    for i=1:TargetNum
        minAngle = mod(Centerangle(1,1) + pi + 0*pi/180,2*pi);
        Xtemp = StaticObstaclePos(1,i,startj(i));
        Ytemp = StaticObstaclePos(2,i,startj(i));
        for zhinum = 1:500
            j = j + 1;
            StaticObstaclePos(1,i,j) = Xtemp - zhinum*cos(minAngle) * 0.2; 
            StaticObstaclePos(2,i,j) = Ytemp - zhinum*sin(minAngle) * 0.2; 
            XL1(i,zhinum) = StaticObstaclePos(1,i,j);
            YL1(i,zhinum) = StaticObstaclePos(2,i,j);
        end
        minAngle = mod(Centerangle(1,1) + pi - 0*pi/180,2*pi);
        Xtemp = StaticObstaclePos(1,i,endj(i));
        Ytemp = StaticObstaclePos(2,i,endj(i));
        for zhinum = 1:500
            j = j + 1;
            StaticObstaclePos(1,i,j) = Xtemp -  zhinum*cos(minAngle) * 0.2; 
            StaticObstaclePos(2,i,j) = Ytemp -  zhinum*sin(minAngle) * 0.2; 
            XL2(i,zhinum) = StaticObstaclePos(1,i,j);
            YL2(i,zhinum) = StaticObstaclePos(2,i,j);
        end
    end
    plot(XL1,YL1,'.k','MarkerSize',3);
    plot(XL2,YL2,'.k','MarkerSize',3);
end
plot(UavPos1(1,:),UavPos1(2,:),'r.','MarkerSize',2);
plot(UavPos2(1,:),UavPos2(2,:),'.','color',[0/255 153/255 51/255],'MarkerSize',2);
xticks(-200:100:200);
yticks(-200:100:200);
axis([-215 215 -215 215]);
% set(gca,'xticklabel',[]);


