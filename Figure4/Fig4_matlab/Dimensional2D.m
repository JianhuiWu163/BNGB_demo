function Dimensional2D(omin,omax,num)
% »­¶à¸öÔ²»¡
ndelta = omax/num;
for n=ndelta:ndelta:omax
	[xnn,ynn] = circle(0,0,n,50);
	plot(xnn,ynn,'k:');%,'LineWidth',1);
	hold on;
end
% »­ºáÊúÏß
j = 1;
for n = omin:omax/25:omax
	line_x1(j) = n;
	line_y1(j) = 0;
	line_x2(j) = 0;
	line_y2(j) = n;
	j = j + 1;
end
plot(line_x1,line_y1,'k:',line_x2,line_y2,'k:');%,'LineWidth',1);
