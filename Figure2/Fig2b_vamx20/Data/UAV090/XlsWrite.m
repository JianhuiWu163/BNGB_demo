close all; clc; clear all;
TxtFile='F:\1. 无人机蜂群\program\Figure2\Fig2bc_vamx20\UAV002\Dij_UAV002_Num0001.txt';
XlsFile='F:\1. 无人机蜂群\program\Figure2\Fig2bc_vamx20\Dij_UAV001.xlsx';

for i=1:26
    serialnum(i)=char(i+64);
end
Uavs=90;
% for Uavs = 3:100 
	tempVar = floor(Uavs/100);
    TxtFile(46) = tempVar + 48;
	TxtFile(57) = tempVar + 48;
	XlsFile(50) = tempVar + 48;
    
    TxtFile(47) = floor(mod(Uavs,100)/10) + 48;%floor((Uavs - tempVar*100)/10) + 48;
	TxtFile(58) = floor(mod(Uavs,100)/10) + 48;
	XlsFile(51) = floor(mod(Uavs,100)/10) + 48;
    
    TxtFile(48) = mod(Uavs,10) + 48;
	TxtFile(59) = mod(Uavs,10) + 48;
	XlsFile(52) = mod(Uavs,10) + 48;
	for Number=1:1000
        ShowValue = [Uavs,Number]
		tempVar = floor(Number/1000);
		TxtFile(64) = tempVar + 48;
		
		tempVar = floor(mod(Number,1000)/100);
		TxtFile(65) = tempVar + 48;
		
		tempVar = floor(mod(Number,100)/10);
		TxtFile(66) = tempVar + 48;
		
		TxtFile(67) = mod(Number,10)+48;
		
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


