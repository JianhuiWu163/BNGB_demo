%********************************************************************************************%
clc;close all;clear all;
rand('state',sum(100*clock));
filename = 'Target01.xlsx';

xMin = -200; xMax = 200; 
yMin = -200; yMax = 200;

UAV_TotalNum = 1000;		% ���˻�����

CollisionNum = 0;   	% ���ϼ���
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

% ����Ŀ���
TargetNum = 10;                             % Ŀ������
TargetPos = zeros(2,TargetNum);           	% Ŀ��x���ꡢy����
TargetPos(1,1) = 0.55 * xMax * cos(2*pi/32); 
TargetPos(2,1)= 0.55 * xMax * sin(2*pi/32);
TargetPos(1,2) = 0.5 * xMax * cos(9*pi/32); 
TargetPos(2,2)= 0.5 * xMax * sin(9*pi/32);
TargetPos(1,3) = 0.65 * xMax * cos(19*pi/32); 
TargetPos(2,3)= 0.65 * xMax * sin(19*pi/32);
TargetPos(1,4) = 0.8 * xMax * cos(26*pi/32); 
TargetPos(2,4)= 0.8 * xMax * sin(26*pi/32);
TargetPos(1,5) = 0.53 * xMax * cos(29*pi/32); 
TargetPos(2,5)= 0.53 * xMax * sin(29*pi/32);

TargetPos(1,6) = 0.4 * xMax * cos(35*pi/32); 
TargetPos(2,6)= 0.4 * xMax * sin(35*pi/32);
TargetPos(1,7) = 0.6 * xMax * cos(40*pi/32); 
TargetPos(2,7)= 0.6 * xMax * sin(40*pi/32);
TargetPos(1,8) = 0.65 * xMax * cos(48*pi/32); 
TargetPos(2,8)= 0.65 * xMax * sin(48*pi/32);
TargetPos(1,9) = 0.85 * xMax * cos(56*pi/32); 
TargetPos(2,9)= 0.85 * xMax * sin(56*pi/32);
TargetPos(1,10) = 0.8 * xMax * cos(62*pi/32); 
TargetPos(2,10)= 0.8 * xMax * sin(62*pi/32);


% ̽��Ŀ��
SimilarFlag = 0;
SelTargCount = 0;
TaskDuration = 200;
TaskCount = 10;
ReachMaxRadius = 0.8;       % ����Ŀ����������뾶

WorkState = zeros(1,UAV_TotalNum);
Sel_TargetPos = zeros(2,TargetNum);     	% �洢̽�⵽��Ŀ������
tempVar_Prob = zeros(UAV_TotalNum,TargetNum);
FinishFlag = zeros(UAV_TotalNum,TargetNum);
Execu_TargetPos = zeros(2,UAV_TotalNum); 
tempPosBuff = zeros(2,TargetNum);
tempVar_Prob = zeros(UAV_TotalNum,TargetNum);
ReselectFlag = zeros(1,UAV_TotalNum);

tempFinishFlag = zeros(1,TargetNum);
detectFirstFlag = zeros(1,TargetNum);
DataBuffer2 = zeros(TargetNum,3);

Dis = zeros(1,UAV_TotalNum);
last_Dis = 0;
ScouterPos = zeros(3,UAV_TotalNum);     	% ��������˻���x���ꡢy���ꡢ����
[ScouterPos(1,:),ScouterPos(2,:),ScouterPos(3,:)] = Scouter_PosInitial(0,0,d,UAV_TotalNum);	% ���������������
Last_ScouterPos = ScouterPos;

%********************************************************************************************%
figure(1)
% set(gcf,'Position',[50 50 180 180]);    % ͼƬ��С 
% set(gca,'Position',[.15 .12 .8 .8]);    % ��������ռ����
Dimensional2D(xMin,xMax,5);            	% ����ά����
axis([xMin xMax yMin yMax]);
set(gca,'Fontsize',8);                 	% ����̶������С
hold on;
for i=1:TargetNum  
	plot(TargetPos(1,i),TargetPos(2,i),'sb');
	text(TargetPos(1,i),TargetPos(2,i),num2str(i),'Color','red','Fontsize',9);
end

[StaticObstaclePos(1,:),StaticObstaclePos(2,:)]= TestMap();

SONum = length(StaticObstaclePos(1,:));
smu = ones(UAV_TotalNum,SONum);	% ��̬�ϰ����ĳ�ͻ״̬��1��ȫ��0С�ڰ�ȫ����
ObsAngle = zeros(UAV_TotalNum,SONum);
ObsBuffer = zeros(2,UAV_TotalNum,SONum);
ObsBuferfNum = zeros(1,UAV_TotalNum); 

%********************************************************************************************%
for i=1:UAV_TotalNum
    plot(ScouterPos(1,i),ScouterPos(2,i),'xr','LineWidth',1.5);
    text(ScouterPos(1,i),ScouterPos(2,i),num2str(i),'Fontsize',7);
end

Theta_1s = zeros(1,UAV_TotalNum);
Xtemp = 0;
Ytemp = 0;

CollisionFlag = 0;    %����
BeforePos = zeros(3,UAV_TotalNum); 
ContinumFlag = zeros(1,UAV_TotalNum);
infFlag = zeros(1,UAV_TotalNum);
KeepFlag = zeros(1,UAV_TotalNum);
FlagState = zeros(1,UAV_TotalNum);
BiasAngle = 0;

OffsetDire = ones(1,UAV_TotalNum);  % ������ñ��Ϸ���
for i=1:UAV_TotalNum
    if(rand < 0.5)
        OffsetDire(1,i) = -1;
    end
end
%********************************************************************************************%
SimulationFlag = 1;
for iters = 1:30000000
    Dimensional2D(xMin,xMax,5);            	% ����ά����
    axis([xMin xMax yMin yMax]);
    set(gca,'Fontsize',8);                 	% ����̶������С
    hold on;
%   plot(TargetPos(1,:),TargetPos(2,:),'sb');
	for i=1:TargetNum
        if(sum(FinishFlag(:,i))<TaskCount)
            plot(TargetPos(1,i),TargetPos(2,i),'sb');
            text(TargetPos(1,i),TargetPos(2,i),num2str(i),'Color','red','Fontsize',9);
        end
    end
	[StaticObstaclePos(1,:),StaticObstaclePos(2,:)]= TestMap();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for k=1:TargetNum
        if((sum(FinishFlag(:,k))>=TaskCount)&&(tempFinishFlag(k)==0))
            DataBuffer2(k,3) = iters - 1;
            tempFinishFlag(k) = 1;
        end
    end
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Ŀ��������ɣ�ȡ���������˻�ִ�и�Ŀ������
	nFinishCount = 0;
	for k=1:TargetNum
		if(sum(FinishFlag(:,k))>=TaskCount)
			for i=1:UAV_TotalNum
				if((WorkState(i)==1)&&(Execu_TargetPos(1,i)==TargetPos(1,k))&&(Execu_TargetPos(2,i)==TargetPos(2,k)))
					WorkState(i) = 0;
				end
			end
			nFinishCount = nFinishCount + 1;
		end
	end
	
 	if(nFinishCount>=TargetNum)
		break;	% �˳�����
	end
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for k=1:TargetNum
		for i=1:UAV_TotalNum
			if((WorkState(i)==1)&&(Execu_TargetPos(1,i)==TargetPos(1,k))&&(Execu_TargetPos(2,i)==TargetPos(2,k)))
				DataBuffer1(iters,i,k) = 1;
			else
				DataBuffer1(iters,i,k) = 0;
			end
		end
	end
	%****************************************************************************************%
	for i=1:UAV_TotalNum
		switch WorkState(i)
			%================================================================================%
			% Ŀ����Ѱ
			case 0	
				if(mod(iters,mdelT)==0)
					if((prod(mu(i,:),2)==1)&&(prod3(smu(i,:))==1))
						BoundaryFlag = 1;
						EndlessLoop_num = 0;
						while(BoundaryFlag)
							ScouterPos(3,i) = 2 * pi * rand;
							
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
						ScouterPos(3,i)= Last_ScouterPos(3,i) + pi;
						BoundaryFlag = 0;
					end
				else
					if((prod(mu(i,:),2)==1)&&(prod3(smu(i,:))==1))
						ScouterPos(3,i) = Last_ScouterPos(3,i);
						if(sqrt(Xtemp^2 + Ytemp^2) > xMax)
							ScouterPos(3,i) = Theta_1s(i);
						end
					else
						ScouterPos(3,i) = Last_ScouterPos(3,i) + pi;
					end
				end
				ScouterPos(3,i) = mod(ScouterPos(3,i),2*pi);
				ScouterPos(1,i) = Last_ScouterPos(1,i) + vi * delta_t * cos(ScouterPos(3,i));  
				ScouterPos(2,i) = Last_ScouterPos(2,i) + vi * delta_t * sin(ScouterPos(3,i));
				   
				plot(ScouterPos(1,i),ScouterPos(2,i),'xr','LineWidth',1.5);
				text(ScouterPos(1,i),ScouterPos(2,i),num2str(i),'Fontsize',7);
				hold on;
				
				xPosBuffer(iters,i) = ScouterPos(1,i);
				yPosBuffer(iters,i) = ScouterPos(2,i);
			%================================================================================%
			% ����ִ��
			case 1	
				if(mod(iters,mdelT)==0)
					last_Dis = Dis(1,i);
					Dis(1,i) = sqrt((Execu_TargetPos(1,i)-ScouterPos(1,i))^2 + (Execu_TargetPos(2,i)-ScouterPos(2,i))^2);
					if((prod(mu(i,:),2)==1)&&(prod3(smu(i,:))==1))
						% ���˻���Ŀ���Ƕ�
						if(Dis(1,i)>0)
							if((Execu_TargetPos(2,i)-ScouterPos(2,i))>=0)
								TargetAngle = acos((Execu_TargetPos(1,i)-ScouterPos(1,i))/Dis(1,i));
							else
								TargetAngle = 2*pi - acos((Execu_TargetPos(1,i)-ScouterPos(1,i))/Dis(1,i));
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
							Dobs = sqrt((ScouterPos(1,i)-StaticObstaclePos(1,n))^2 + (ScouterPos(2,i)-StaticObstaclePos(2,n))^2);
							if(Dobs <= DetectRadius)
								StaticObstacleFlag = 1;
								StaticObstacleNumber = StaticObstacleNumber + 1;
								if(Dobs>0)
									if((StaticObstaclePos(2,n)-ScouterPos(2,i))>=0)
										ObsAngle(i,StaticObstacleNumber) = acos((StaticObstaclePos(1,n)-ScouterPos(1,i))/Dobs);
									else
										ObsAngle(i,StaticObstacleNumber) = 2*pi - acos((StaticObstaclePos(1,n)-ScouterPos(1,i))/Dobs);
									end
								else
									ObsAngle(i,StaticObstacleNumber) = 0;
								end
								% ���ӷ�Χ�ڣ��ж��ϰ����Ƿ��谭���˻�����Ŀ���
								if(((abs(TargetAngle - ObsAngle(i,StaticObstacleNumber))<= (1*pi/180))&&(Dobs<Dis(1,i)))||(Dis(1,i)>last_Dis))  % 5*pi/180�Ƿ�ֹ����ʱ�����õ��ϰ���㲻���ܼ�
									TarObsLineFlag = 1;			% ����TarObsLineFlag���ֵ�����
								end
								% �洢��̽�⵽���ϰ���
								RepeatFlag = 0;
								if(ObsBuferfNum(i)>0)
									for j=1:ObsBuferfNum(i)
										if((ObsBuffer(1,i,j)==StaticObstaclePos(1,n))&&(ObsBuffer(2,i,j)==StaticObstaclePos(2,n)))
											RepeatFlag = 1;		% �ж�����֪�ϰ���������ͬ�����洢 
											break;
										end
									end
								end
								if(RepeatFlag==0)		% �޳���ͬ���ϵ����꣬�Դ˼��ٴ洢�ռ�
									ObsBuferfNum(i) = ObsBuferfNum(i) + 1;
									ObsBuffer(1,i,ObsBuferfNum(i)) = StaticObstaclePos(1,n);	% ��֪�ϰ���
									ObsBuffer(2,i,ObsBuferfNum(i)) = StaticObstaclePos(2,n);
								end
							end
						end
						% �ж���֪�ϰ����Ƿ��赲���˻���Ŀ���
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
								if((abs(TargetAngle - Temp)<= (1*pi/180))&&(Dobs<Dis(1,i)))  	% 5*pi/180�Ƿ�ֹ����ʱ�����õ��ϰ���㲻���ܼ�
									TarObsLineFlag = 1;                                     % ����TarObsLineFlag���ֵ�����
									break;
								end
							end
						end                        
						%=====================================================================%
						if((StaticObstacleFlag==1)&&(TarObsLineFlag==1))
							% ������任--y��ָ��Ŀ���
							if(Execu_TargetPos(2,i)>=0)
								BiasAngle = acos(Execu_TargetPos(1,i)/sqrt(Execu_TargetPos(1,i)^2+Execu_TargetPos(2,i)^2));
                          	else
								BiasAngle = 2*pi - acos(Execu_TargetPos(1,i)/sqrt(Execu_TargetPos(1,i)^2+Execu_TargetPos(2,i)^2));
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
% 										MaxObsAngle = ObsAngle(i,j)+(BiasAngle - pi/2);
                                        MaxObsAngle = ObsAngle(i,j)+BiasAngle;
										break;
									end
									if(j==(StaticObstacleNumber-1))
% 										MaxObsAngle = ObsAngle(i,StaticObstacleNumber)+(BiasAngle - pi/2);
                                        MaxObsAngle = ObsAngle(i,StaticObstacleNumber)+BiasAngle;
									end
								end
								ScouterPos(3,i) = MaxObsAngle + OffsetDire(1,i) * (pi/18 + rand*pi/6);
							elseif(OffsetDire(1,i)==-1) %˳ʱ��
								if((ObsAngle(i,StaticObstacleNumber)>350*pi/180)&&(ObsAngle(i,1)<10*pi/180))
									% ���Ƕ��Ƿ�����
									for j=StaticObstacleNumber:-1:2
										if(abs(ObsAngle(i,j) - ObsAngle(i,j-1))>(20*pi/180))    % 0�ȸ����ڽ�����λƫ��ϴ�����ѡ��20*pi/180
% 											MinObsAngle = ObsAngle(i,j)+(BiasAngle - pi/2);
                                            MinObsAngle = ObsAngle(i,j)+BiasAngle;
											break;
										end
									end
								else
% 									MinObsAngle = ObsAngle(i,1)+(BiasAngle - pi/2);
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
								if(Dis(1,i)<ReachMaxRadius)
									for k=1:TargetNum
										if((Execu_TargetPos(1,i)==TargetPos(1,k))&&(Execu_TargetPos(2,i)==TargetPos(2,k)))
											FinishFlag(i,k) = 1;
											WorkState(i) = 0;
										end
									end
								end
							end
						end
							
						% �߽�����(ֻ�м��1s���ܵõ�����)
						Xtemp = Last_ScouterPos(1,i) + vi * Tmax * cos(ScouterPos(3,i));
						Ytemp = Last_ScouterPos(2,i) + vi * Tmax * sin(ScouterPos(3,i));
						if(sqrt(Xtemp^2 + Ytemp^2) > xMax)
							if(Last_ScouterPos(2,i)<=0)
								ScouterPos(3,i) = acos(-Last_ScouterPos(1,i)/sqrt(Last_ScouterPos(1,i)^2+Last_ScouterPos(2,i)^2));
							else
								ScouterPos(3,i) = 2*pi - acos(-Last_ScouterPos(1,i)/sqrt(Last_ScouterPos(1,i)^2+Last_ScouterPos(2,i)^2));
							end
							Xtemp = Last_ScouterPos(1,i) + vi * Tmax * cos(ScouterPos(3,i));
							Ytemp = Last_ScouterPos(2,i) + vi * Tmax * sin(ScouterPos(3,i));
							Theta_1s(i) = ScouterPos(3,i);
						end
					else
						ScouterPos(3,i)= Last_ScouterPos(3,i) + pi;
						BoundaryFlag = 0;
						CollisionFlag = CollisionFlag + 1;		% ����
					end
				else
					if((prod(mu(i,:),2)==1)&&(prod3(smu(i,:))==1))
						ScouterPos(3,i) = Last_ScouterPos(3,i);
						if(sqrt(Xtemp^2 + Ytemp^2) > xMax)
							ScouterPos(3,i) = Theta_1s(i);
						end
					else
						ScouterPos(3,i) = Last_ScouterPos(3,i) + pi;
					end

					%%����Ŀ�ĵ���ͣ��������
					if(sqrt((Execu_TargetPos(1,i)-ScouterPos(1,i))^2 + (Execu_TargetPos(2,i)-ScouterPos(2,i))^2)< ReachMaxRadius)
						for k=1:TargetNum
							if((Execu_TargetPos(1,i)==TargetPos(1,k))&&(Execu_TargetPos(2,i)==TargetPos(2,k)))
								FinishFlag(i,k) = 1;
								WorkState(i) = 0;
							end
						end
					end
				end
				ScouterPos(3,i) = mod(ScouterPos(3,i),2*pi);
				ScouterPos(1,i) = Last_ScouterPos(1,i) + vi * delta_t * cos(ScouterPos(3,i));  
				ScouterPos(2,i) = Last_ScouterPos(2,i) + vi * delta_t * sin(ScouterPos(3,i));
				   
				plot(ScouterPos(1,i),ScouterPos(2,i),'xr','LineWidth',1.5);					
				text(ScouterPos(1,i),ScouterPos(2,i),num2str(i),'Fontsize',7);
				hold on;
				
				xPosBuffer(iters,i) = ScouterPos(1,i);
				yPosBuffer(iters,i) = ScouterPos(2,i);
		end		
	end
    Last_ScouterPos = ScouterPos;
    if(mod(iters,mdelT)==0)
        BeforePos = ScouterPos;
    end
	
	%*********************************************************************%
	% ̽��Ŀ��
	%*********************************************************************%
	SelTargCount = 0;
	for i=1:UAV_TotalNum
		for k=1:TargetNum
			if((FinishFlag(i,k)~=1)&&(sum(FinishFlag(:,k))<TaskCount))
				RadiusTemp = sqrt((ScouterPos(1,i)-TargetPos(1,k))^2 + (ScouterPos(2,i)-TargetPos(2,k))^2);
				if((RadiusTemp<=DetectRadius)&&((WorkState(i)==0)||((WorkState(i)==1)&&(Execu_TargetPos(1,i)~=TargetPos(1,k))&&(Execu_TargetPos(2,i)~=TargetPos(2,k)))))
					if(SelTargCount>0)
						for j=1:SelTargCount
							if((Sel_TargetPos(1,j)==TargetPos(1,k))&&(Sel_TargetPos(2,j)==TargetPos(2,k)))
								SimilarFlag = 1;
								break;
							end
						end
						if(SimilarFlag==0)
							SelTargCount = SelTargCount + 1;
							Sel_TargetPos(1,SelTargCount) = TargetPos(1,k);
							Sel_TargetPos(2,SelTargCount) = TargetPos(2,k);
						end
						SimilarFlag = 0;
					else
						SelTargCount = SelTargCount + 1;
						Sel_TargetPos(1,SelTargCount) = TargetPos(1,k);
						Sel_TargetPos(2,SelTargCount) = TargetPos(2,k);
					end
					%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					% �洢��Ŀ�귢��ʱ�ĵ�������
					if(detectFirstFlag(k)==0)
						DataBuffer2(k,1) = i;			% ���˻����
						DataBuffer2(k,2) = iters + 1;	% ����Ŀ�굱ǰ������
						detectFirstFlag(k) = 1;
					end
					%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				end
			end
		end
	end
	
	% ��ʼ��
	for i=1:UAV_TotalNum
		ReselectFlag(i) = 0;
        for n=1:TargetNum
			tempVar_Prob(i,n) = 0;
        end
	end
	for i=1:TargetNum
        tempPosBuff(1,i) = 0;
        tempPosBuff(2,i) = 0;
	end
	
	if(SelTargCount>0)
		for i=1:UAV_TotalNum
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			% ȥ�����˻��ѵ�����Ŀ�ĵ�
			tempCount = 0;
			if(sum(FinishFlag(i,:))>0)
				for n=1:SelTargCount
					tempFlag = 0;
					for mk=1:TargetNum
						if((FinishFlag(i,mk)==1)&&(Sel_TargetPos(1,n)==TargetPos(1,mk))&&(Sel_TargetPos(2,n)==TargetPos(2,mk)))
							tempFlag = 1;
						end							
					end
						
					if(tempFlag==0)
						tempCount = tempCount + 1;
						tempPosBuff(1,tempCount)=Sel_TargetPos(1,n);
						tempPosBuff(2,tempCount)=Sel_TargetPos(2,n);
					end
				end					
            else
                tempCount = SelTargCount;
				tempPosBuff = Sel_TargetPos;
			end
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	
			if(tempCount>0)
				for k=1:tempCount
					RadiusTemp = sqrt((ScouterPos(1,i)-tempPosBuff(1,k))^2 + (ScouterPos(2,i)-tempPosBuff(2,k))^2);
					if(RadiusTemp<=DetectRadius)
						if(WorkState(i)==0)
							tempVar_Prob(i,k) = 1;
						elseif((WorkState(i)==1)&&(Execu_TargetPos(1,i)~=tempPosBuff(1,k))&&(Execu_TargetPos(2,i)~=tempPosBuff(2,k)))
							tempVar_Prob(i,k) = 1;
							ReselectFlag(i) = 1;
							if(sqrt((ScouterPos(1,i)-Execu_TargetPos(1,i))^2 + (ScouterPos(2,i)-Execu_TargetPos(2,i))^2)<=DetectRadius)
								ReselectFlag(i) = 2;
							end
						else
							tempVar_Prob(i,k) = 0;
						end
					else
						tempVar_Prob(i,k) = 0;
					end
				end
				%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				if((WorkState(i)==0)||(ReselectFlag(i)>=1))	
					%----------------------------------------------------%
					% ������ʱ��������Ӧ��
					Fs(1) = 0;
					for n=2:(tempCount+1)
						Fs(n) = 1/(1+TaskDuration);
					end
					Fs(1) = rand * max(Fs);
					
					% �����о��������Ӧ��
					Fd(1) = 0;
					for n=2:(tempCount+1)
						Fd(n) = 1/(1+sqrt((tempPosBuff(1,n-1)-ScouterPos(1,i))^2 + (tempPosBuff(2,n-1)-ScouterPos(2,i))^2));
					end
					Fd(1) = rand * max(Fd);
					%----------------------------------------------------%	
					% ����ѡ�����
					if(sum(tempVar_Prob(i,:)~=0))   % WorkState(i)=1�龰��
						if(ReselectFlag(i)>=2)  % ���޸�֪��Χ����ԭĿ�����Ŀ��ʱ������ԭĿ�겻��
 							pds(1) = 1;         % ԭĿ������		
 							for n=2:(tempCount+1)
 								pds(n) = 0;     % tempVar_Prob(i,n-1)/(sum(tempVar_Prob(i,:))+1);
 							end
                        else                    % ���޸�֪��Χ�ڽ�����Ŀ��ʱ��ѡ����Ŀ��
							pds(1) = 0;
							for n=2:(tempCount+1)
								pds(n) = tempVar_Prob(i,n-1)/sum(tempVar_Prob(i,:));
							end
						end
                    else   % WorkState(i)=0�龰��
						tempRand = rand;
						for n=1:(tempCount+1)
							pds(n) = tempRand * Fd(n)/sum(Fd) + (1 - tempRand) * Fs(n)/sum(Fs);
						end 
					end				
					%----------------------------------------------------%	
					% ����ѡ��Ŀ������
					Selj = find(rand<=cumsum(pds),1,'first');	
					
					%----------------------------------------------------%			
					if(Selj>=2)		% Selj=1��ʾ����ԭ����
						Execu_TargetPos(1,i)=tempPosBuff(1,Selj-1);
						Execu_TargetPos(2,i)=tempPosBuff(2,Selj-1);
						WorkState(i) = 1;
% 					else
% 						WorkState(i) = 0;	
					end
				end
            end
		end
	end
    %*********************************************************************%
    % ��������˻����ͻ״̬
	%*********************************************************************%
	DijBefore = inf;
    for i=1:UAV_TotalNum
        for j=1:UAV_TotalNum
			if(j==i)
				Dij = 0;
				mu(i,j) = 1;
            else
				Dij = sqrt((ScouterPos(1,i)-ScouterPos(1,j))^2 + (ScouterPos(2,i)-ScouterPos(2,j))^2);
				if(Dij < DijBefore)
					DijBefore = Dij;
				end
				
				if(Dij > d)
					mu(i,j) = 1;
                else
					mu(i,j) = 0;
                end
         	end
        end
		
		for	k=1:SONum	
			Dij = sqrt((ScouterPos(1,i)-StaticObstaclePos(1,k))^2 + (ScouterPos(2,i)-StaticObstaclePos(2,k))^2);
            if(Dij < DijBefore)
				DijBefore = Dij;
			end
			
			if(Dij > d)
				smu(i,k) = 1;
			else
				smu(i,k) = 0;
			end
		end
    end
	NearNeigDis(iters,1) = DijBefore;
	pause(0.001); 
    hold off;
end

xlswrite('xPosBuffer.xlsx',xPosBuffer,'Sheet1');
xlswrite('yPosBuffer.xlsx',yPosBuffer,'Sheet1');
xlswrite('FinishFlag.xlsx',FinishFlag,'Sheet1');
xlswrite('NearNeigDis.xlsx',NearNeigDis,'Sheet1');
xlswrite('DataBuffer2.xlsx',DataBuffer2,'Sheet1');

for k=1:TargetNum
	filename(7) = floor(k/10) + 48;
	filename(8) = mod(k,10) + 48;
	xlswrite(filename,DataBuffer1(:,:,k),'Sheet1');
end



