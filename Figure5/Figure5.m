clc;
close all;
clear all;
xMin = -200; xMax = 200; 
yMin = -200; yMax = 200;

% 生成目标点
TargetNum = 10;                             % 目标总数
TargetPos = zeros(2,TargetNum);           	% 目标x坐标、y坐标
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

%******************************************************************************************%
L= xlsread('F:\1. 无人机蜂群\program\Figure5\Data\DataBuffer2.xlsx');
fp = 'F:\1. 无人机蜂群\program\Figure5\Data\Target01.xlsx';
    
z0 = zeros(1,3300);
for n=1:10
    fp(40) = n/10 + 48;
    fp(41) = mod(n,10) + 48;
    y1 = xlsread(fp);
    for i=L(n,2):L(n,3)
        x(i) = i;
        y(i) = sum(y1(i,:))/1000;   % 换算成百分比 
    end
    figure(n)
    set(gcf,'Position',[50 50 350 60]);%图片大小
    set(gca,'Position',[.05 .17 .9 .7]);%坐标轴所占比例 
%     set(gca,'Position',[.12 .17 .77 .7]);%坐标轴所占比例    
    for m=1:10
        plot([L(m,2),L(m,2)],[0,1],':','Color',[0 0.5 0],'LineWidth',0.5);
        hold on;
        plot([L(m,3),L(m,3)],[0,1],'r:','LineWidth',0.5);
    end
    xlim([0 3300]);
    xticks(0:1000:3300);
    yticks(0:0.1:1);
    set(gca,'Fontsize',8);%坐标刻度字体大小
    box off;
    
    y2=fliplr(y(L(n,2):L(n,3)));%最大值反向
    y3=z0(L(n,2):L(n,3));
    y4=[y3 y2];
    x4=[L(n,2):L(n,3) L(n,3):-1:L(n,2)];
    h=fill(x4,y4,'b');
    set(h,'edgealpha',0,'facealpha',0.4);
    plot(x(L(n,2):L(n,3)),y(L(n,2):L(n,3)),'b','LineWidth',0.2);
    set(gca,'xticklabel',[],'yticklabel',[]);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(11)
set(gcf,'Position',[50 50 250 250]);    % 图片大小 
set(gca,'Position',[.15 .12 .8 .8]);    % 坐标轴所占比例
Dimensional2D(xMin,xMax,5);            	% 画二维坐标
xticks(-200:100:200);
yticks(-200:100:200);
axis([-210 210 -210 210]);
set(gca,'Fontsize',8);                 	% 坐标刻度字体大小
set(gca,'FontName','Times New Roman');
hold on;

[StaticObstaclePos(1,:),StaticObstaclePos(2,:)]= TestMap();

Co = [0         0       1;      
      0         0.5     0;      
      0.75      0.75    0;      
      0         0.75    0.75;        
      0.75      0       0.75;      
      1         0       0;      
      0         1       0;
      0.75      0.25    0.75;      
      0.75      1       0;      
      1         0.6     0]; 
% xData= xlsread('F:\1. 无人机蜂群\program\Figure5\Data\xPosBuffer.xlsx');
% yData= xlsread('F:\1. 无人机蜂群\program\Figure5\Data\yPosBuffer.xlsx');
% for n=10:-1:1
%     col = L(n,1);
%     row = L(n,2);
%     plot(xData(1:row,col),yData(1:row,col),'Color',Co(n,:),'LineWidth',1.5);
%     hold on
% end

for i=1:TargetNum  
	plot(TargetPos(1,i),TargetPos(2,i),'or','MarkerSize',4,'MarkerFaceColor','r')
    if(i==10)
        text(TargetPos(1,i)-13,TargetPos(2,i)-15,num2str(i),'Color','blue','Fontsize',8,'FontName','Times New Roman');
    else
        text(TargetPos(1,i)-6,TargetPos(2,i)-15,num2str(i),'Color','blue','Fontsize',8,'FontName','Times New Roman');
    end
end
set(gca,'xticklabel',[],'yticklabel',[]);
%%******************************************************************************************%
figure(12)
yData = xlsread('F:\1. 无人机蜂群\program\Figure5\Data\NearNeigDis.xlsx');
yData = yData';
set(gcf,'Position',[50 50 250 110]);%图片大小
set(gca,'Position',[.15 .2 .8 .75]);    % 坐标轴所占比例

N = length(yData);   %数据长度
x = 1:N;
c = 1./(yData-0.8);
sz = 1.6;
scatter(x,yData,sz,c,'filled');
colormap(jet)   %查阅colormap函数改变颜色变化趋势
axis([1 3400 0.5 1.5]);
set(gca,'Fontsize',8);	% 坐标刻度字体大小
set(gca,'FontName','Times New Roman');
hold on;
safeDis = 0.8 * ones(1,N);
plot(x,safeDis,'-.r');
% text(-300,0.8,'0.8','Fontsize',8,'FontName','Times New Roman');
box off;
xticks(0:1000:3000);
yticks(0.5:0.5:1.5);
set(gca,'xticklabel',[],'yticklabel',[]);
% %******************************************************************************************%
% figure(13)
% yData = xlsread('F:\1. 无人机蜂群\program\Figure5\Data\DataBuffer2.xlsx');
% yData = yData./1000;
% set(gcf,'Position',[50 50 250 110]);%图片大小
% set(gca,'Position',[.15 .2 .8 .67]);    % 坐标轴所占比例
% x=1:10;
% plot(x,yData(:,2),'-o','Color',[0 0.5 0],'LineWidth',0.6,'MarkerSize',3,'MarkerFaceColor',[0 0.5 0]);
% xticks(1:10);
% yticks(0:1:3);
% axis([0 11 0 3.8]);
% set(gca,'Fontsize',8);	% 坐标刻度字体大小
% set(gca,'FontName','Times New Roman');
% hold on;
% % plot(x,yData(:,3),'-^','Color',[0.85 0.33 0.1],'LineWidth',0.6,'MarkerSize',3,'MarkerFaceColor',[0.85 0.33 0.1]);
% plot(x,yData(:,3),'r-^','LineWidth',0.6,'MarkerSize',3,'MarkerFaceColor',[0.85 0.33 0.1]);
% text(0,4.2,'\times 10^3','Fontsize',8,'FontName','Times New Roman');
% % grid on;
% box off;


