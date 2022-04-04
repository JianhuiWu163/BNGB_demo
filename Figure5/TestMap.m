function [Xobs,Yobs]= TestMap()
% clc;clear all;close all;
Xobs = zeros(1,2350);
Yobs = zeros(1,2350);

[Xtemp,Ytemp] = circle(-100,110,15,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
hold on;
Xobs(1:100)=Xtemp(1:100);
Yobs(1:100)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(0,140,15,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(101:200)=Xtemp(1:100);
Yobs(101:200)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(-130,-80,15,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(201:300)=Xtemp(1:100);
Yobs(201:300)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(0,140,15,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(301:400)=Xtemp(1:100);
Yobs(301:400)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(80,10,15,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(401:500)=Xtemp(1:100);
Yobs(401:500)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(30,-110,15,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(501:600)=Xtemp(1:100);
Yobs(501:600)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(-50,-40,15,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(601:700)=Xtemp(1:100);
Yobs(601:700)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(130,-70,15,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(701:800)=Xtemp(1:100);
Yobs(701:800)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(150,0,15,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(801:900)=Xtemp(1:100);
Yobs(801:900)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(120,90,11,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(901:1000)=Xtemp(1:100);
Yobs(901:1000)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(90,-113,11,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1001:1100)=Xtemp(1:100);
Yobs(1001:1100)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(-60,-120,11,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1101:1200)=Xtemp(1:100);
Yobs(1101:1200)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(-45,90,11,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1201:1300)=Xtemp(1:100);
Yobs(1201:1300)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(-120,20,11,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1301:1400)=Xtemp(1:100);
Yobs(1301:1400)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(99,-90,8,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1401:1500)=Xtemp(1:100);
Yobs(1401:1500)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(130,20,8,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1501:1600)=Xtemp(1:100);
Yobs(1501:1600)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(40,70,8,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1601:1700)=Xtemp(1:100);
Yobs(1601:1700)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(-100,60,8,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1701:1800)=Xtemp(1:100);
Yobs(1701:1800)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(-80,-60,8,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1801:1900)=Xtemp(1:100);
Yobs(1801:1900)=Ytemp(1:100);

[Xtemp,Ytemp] = circle(-80,30,6,25);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1901:1950)=Xtemp(1:50);
Yobs(1901:1950)=Ytemp(1:50);

[Xtemp,Ytemp] = circle(60,110,6,25);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(1951:2000)=Xtemp(1:50);
Yobs(1951:2000)=Ytemp(1:50);

[Xtemp,Ytemp] = circle(90,-40,3,25);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(2001:2050)=Xtemp(1:50);
Yobs(2001:2050)=Ytemp(1:50);

[Xtemp,Ytemp] = circle(-140,-30,3,25);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(2051:2100)=Xtemp(1:50);
Yobs(2051:2100)=Ytemp(1:50);

[Xtemp,Ytemp] = circle(-15,-140,3,25);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(2101:2150)=Xtemp(1:50);
Yobs(2101:2150)=Ytemp(1:50);

[Xtemp,Ytemp] = circle(70,-80,3,25);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(2151:2200)=Xtemp(1:50);
Yobs(2151:2200)=Ytemp(1:50);

[Xtemp,Ytemp] = circle(40,-50,3,25);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(2201:2250)=Xtemp(1:50);
Yobs(2201:2250)=Ytemp(1:50);

[Xtemp,Ytemp] = circle(-15,50,3,25);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(2251:2300)=Xtemp(1:50);
Yobs(2251:2300)=Ytemp(1:50);

[Xtemp,Ytemp] = circle(-100,80,3,50);
plot(Xtemp,Ytemp,'k','LineWidth',1.5);
Xobs(2301:2350)=Xtemp(1:50);
Yobs(2301:2350)=Ytemp(1:50);


