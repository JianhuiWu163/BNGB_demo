close all; clc; clear all;
TxtFile='F:\1. 无人机蜂群\program\Figure2\Fig2bc_vamx20\Data\UAV002\Dij_UAV002_Num0001.txt';
XlsFile='F:\1. 无人机蜂群\program\Figure2\Fig2bc_vamx20\Data\Dij_UAV001.xlsx';

for i=1:26
    serialnum(i)=char(i+64);
end
Uavs=26;
% for Uavs = 3:100 
	tempVar = floor(Uavs/100);
    TxtFile(51) = tempVar + 48;
	TxtFile(62) = tempVar + 48;
	XlsFile(55) = tempVar + 48;
    
    TxtFile(52) = floor(mod(Uavs,100)/10) + 48;%floor((Uavs - tempVar*100)/10) + 48;
	TxtFile(63) = floor(mod(Uavs,100)/10) + 48;
	XlsFile(56) = floor(mod(Uavs,100)/10) + 48;
    
    TxtFile(53) = mod(Uavs,10) + 48;
	TxtFile(64) = mod(Uavs,10) + 48;
	XlsFile(57) = mod(Uavs,10) + 48;
	for Number=1:1000
        ShowValue = [Uavs,Number]
		tempVar = floor(Number/1000);
		TxtFile(69) = tempVar + 48;
		
		tempVar = floor(mod(Number,1000)/100);
		TxtFile(70) = tempVar + 48;
		
		tempVar = floor(mod(Number,100)/10);
		TxtFile(71) = tempVar + 48;
		
		TxtFile(72) = mod(Number,10)+48;
		
		m=textread(TxtFile);
        if(isempty(m)==0)
            if(Number<=26)
                colnum=serialnum(Number);
            elseif((Number>=26)&&(Number<=702))
                colnum=strcat(serialnum(ceil(Number/26)-1),serialnum(mod(Number-1,26)+1));
            else
                colnum=strcat(serialnum(ceil(Number/702)-1),serialnum(ceil(Number/26)-27),serialnum(mod(Number-1,26)+1));
            end
            xlswrite(XlsFile,m,1,colnum);
        end
	end
% end


