close all; clc; clear all;
TxtFile='F:\1. 无人机蜂群\program\Figure4\Fig4_1000\Data\Iters_UAV000.txt';
XlsFile='F:\1. 无人机蜂群\program\Figure4\Fig4_1000\Data\Iters_UAV.xlsx';

for i=1:26
    serialnum(i)=char(i+64);
end

for Uavs=1:100
	ShowValue = Uavs
	tempVar = floor(Uavs/100);
	TxtFile(53) = tempVar + 48;
		
	tempVar = floor(mod(Uavs,100)/10);
	TxtFile(54) = tempVar + 48;
		
	TxtFile(55) = mod(Uavs,10)+48;
		
	m=textread(TxtFile);
	if(isempty(m)==0)
        if(Uavs<=26)
            colnum=serialnum(Uavs);
     	elseif((Uavs>26)&&(Uavs<=702))
        	colnum=strcat(serialnum(ceil(Uavs/26)-1),serialnum(mod(Uavs-1,26)+1));
        else
        	colnum=strcat(serialnum(ceil(Uavs/702)-1),serialnum(ceil(Uavs/26)-27),serialnum(mod(Uavs-1,26)+1));
        end
    	xlswrite(XlsFile,m,1,colnum);
	end
end

TxtFile='F:\1. 无人机蜂群\program\Figure4\Fig4_1000\Data\Dij_UAV000.txt';
XlsFile='F:\1. 无人机蜂群\program\Figure4\Fig4_1000\Data\Dij_UAV.xlsx';

for Uavs=1:100
	ShowValue = Uavs
	tempVar = floor(Uavs/100);
	TxtFile(51) = tempVar + 48;
		
	tempVar = floor(mod(Uavs,100)/10);
	TxtFile(52) = tempVar + 48;
		
	TxtFile(53) = mod(Uavs,10)+48;
		
	m=textread(TxtFile);
	if(isempty(m)==0)
        if(Uavs<=26)
            colnum=serialnum(Uavs);
     	elseif((Uavs>26)&&(Uavs<=702))
        	colnum=strcat(serialnum(ceil(Uavs/26)-1),serialnum(mod(Uavs-1,26)+1));
        else
        	colnum=strcat(serialnum(ceil(Uavs/702)-1),serialnum(ceil(Uavs/26)-27),serialnum(mod(Uavs-1,26)+1));
        end
    	xlswrite(XlsFile,m,1,colnum);
	end
end

