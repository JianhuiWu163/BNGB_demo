clc; close all; clear all;
UAVs = 100;
TargetNum = 4;
vt = 20;
% Distance = [200,400,560,780];
Distance = [100,200,300,400];
TotalTasks = [19 29 39 49]; 

Nmax = 1000;
TotalFlyTime = zeros(Nmax,3);
Ed = zeros(Nmax,TargetNum);	% work efficiency
Es = zeros(Nmax,TargetNum);
Eds = zeros(Nmax,TargetNum);

Ud = zeros(Nmax,TargetNum);
Us = zeros(Nmax,TargetNum);
Uds = zeros(Nmax,TargetNum);

for num=1:Nmax
    %========================================================%
    % 依据任务量选择
    %========================================================%
    % 适应度
    for i=1:TargetNum
        Fs(i) = 1+TotalTasks(i);
    end
    % 选择概率
    for i=1:TargetNum
        ps(i) = Fs(i)/sum(Fs);
    end
    % 无人机数
    for i=1:UAVs
        j = find(rand<=cumsum(ps),1,'first');
        Us(num,j) = Us(num,j) + 1;
    end
    
    % 系统总飞行时间
    for i=1:TargetNum
        TotalFlyTime(num,1) = TotalFlyTime(num,1) + Us(num,i) * Distance(i)/vt;
    end
	
    % 工作效率
    for i=1:TargetNum
        if(Us(num,i)>0)
            Es(num,i) = Es(num,i) + TotalTasks(i)/Us(num,i);
        end
    end
    
    %========================================================%
    % 依据飞行距离选择
    %========================================================%
    % 适应度
    for i=1:TargetNum
        Fd(i) = 1/(1+Distance(i));
    end
    % 选择概率
    for i=1:TargetNum
        pd(i) = Fd(i)/sum(Fd);
    end
    % 无人机数
    for i=1:UAVs
        j = find(rand<=cumsum(pd),1,'first');
        Ud(num,j) = Ud(num,j) + 1;
    end

    % 系统总飞行时间
    for i=1:TargetNum
        TotalFlyTime(num,3) = TotalFlyTime(num,3) + Ud(num,i) * Distance(i)/vt;
    end
	
    % 工作效率
    for i=1:TargetNum
        if(Us(num,i)>0)
            Ed(num,i) = Ed(num,i) + TotalTasks(i)/Ud(num,i);
        end
    end

    %========================================================%
    % 依据飞行距离和任务量综合选择
    %========================================================%
    % 无人机数
    for i=1:UAVs
        % 选择概率
        RA = rand;
        for n=1:TargetNum
            pds(n) = RA * Fd(n)/sum(Fd) + (1 - RA) * Fs(n)/sum(Fs);
        end  
        j = find(rand<=cumsum(pds),1,'first');
        Uds(num,j) = Uds(num,j) + 1;
    end

    % 系统总飞行时间
    for i=1:TargetNum
        TotalFlyTime(num,2) = TotalFlyTime(num,2) + Uds(num,i) * Distance(i)/vt;
    end
	
    % 工作效率
    for i=1:TargetNum
        if(Us(num,i)>0)
            Eds(num,i) = Eds(num,i) + TotalTasks(i)/Uds(num,i);
        end
    end
end

for i=1:3
    ATFlyTime(i) = mean(TotalFlyTime(:,i));
end
SDTFT = std(TotalFlyTime,0,1); % standard deviation 

for i=1:TargetNum
    AUd(i) = mean(Ud(:,i));
    AUs(i) = mean(Us(:,i));
	AUds(i) = mean(Uds(:,i));
	
	AEd(i) = mean(Ed(:,i));
    AEs(i) = mean(Es(:,i));
	AEds(i) = mean(Eds(:,i));
end
SDUd = std(Ud,0,1); % standard deviation 
SDUs = std(Us,0,1);
SDUds = std(Uds,0,1);

SDEd = std(Ed,0,1); % standard deviation 
SDEs = std(Es,0,1);
SDEds = std(Eds,0,1);

figure(1);
set(gcf,'Position',[50 50 300 120]);%图片大小
x1 = [0.7,1.7,2.7,3.7];
cbar1 = bar(x1,AUs,0.3,'LineStyle','none');
set(cbar1,'FaceColor',[102/255 153/255 255/255]);
hold on;
axis([0.5 4.5 0 60]);
errorbar(x1,AUs,SDUs,'.k','LineStyle','none');

x2 = [1,2,3,4];
cbar2 = bar(x2,AUds,0.3,'y','LineStyle','none');
set(cbar2,'FaceColor',[255/255 204/255 153/255]);
errorbar(x2,AUds,SDUds,'.k','LineStyle','none');

x3 = [1.3,2.3,3.3,4.3];
cbar3 = bar(x3,AUd,0.3,'y','LineStyle','none');
set(cbar3,'FaceColor',[255/255 204/255 204/255]);
errorbar(x3,AUd,SDUd,'.k','LineStyle','none');
set(gca,'Fontsize',8);%坐标刻度字体大小
box off;

figure(2);
set(gcf,'Position',[50 50 300 115]);%图片大小
x1 = [1,2,3];
y1 = [ATFlyTime(1),0,0];
cbar1 = bar(x1,y1,0.3,'LineStyle','none');
set(cbar1,'FaceColor',[102/255 153/255 255/255]);
set(gca,'xticklabel',[])
hold on;
axis([0.7 3.3 0 2000]);
x0 = 1;
errorbar(x0,ATFlyTime(1),SDTFT(1),'.k','LineStyle','none');

x2 = 2;
cbar2 = bar(x2,ATFlyTime(2),0.3,'LineStyle','none');
set(cbar2,'FaceColor',[255/255 204/255 153/255]);
errorbar(x2,ATFlyTime(2),SDTFT(2),'.k','LineStyle','none');

x3 = 3;
cbar3 = bar(x3,ATFlyTime(3),0.3,'LineStyle','none');
set(cbar3,'FaceColor',[255/255 204/255 204/255]);
errorbar(x3,ATFlyTime(3),SDTFT(3),'.k','LineStyle','none');
set(gca,'Fontsize',8);%坐标刻度字体大小
box off;

%===========================================================%
figure(3);
set(gcf,'Position',[50 50 300 120]);%图片大小
x1 = [0.7,1.7,2.7,3.7];
cbar1 = bar(x1,AEs,0.3,'LineStyle','none');
set(cbar1,'FaceColor',[102/255 153/255 255/255]);
hold on;
axis([0.5 4.5 0 6]);
errorbar(x1,AEs,SDEs,'.k','LineStyle','none');

x2 = [1,2,3,4];
cbar2 = bar(x2,AEds,0.3,'y','LineStyle','none');
set(cbar2,'FaceColor',[255/255 204/255 153/255]);
errorbar(x2,AEds,SDEds,'.k','LineStyle','none');

x3 = [1.3,2.3,3.3,4.3];
cbar3 = bar(x3,AEd,0.3,'y','LineStyle','none');
set(cbar3,'FaceColor',[255/255 204/255 204/255]);
errorbar(x3,AEd,SDEd,'.k','LineStyle','none');
set(gca,'Fontsize',8);%坐标刻度字体大小
box off;


