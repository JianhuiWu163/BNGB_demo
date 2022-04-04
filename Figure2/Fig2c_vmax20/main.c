#include "main.h"

//===========================================================//
void main()
{
	long int i,j,k,n;
	int TempVar=0;	
	double Xtemp = 0;
	double Ytemp = 0;
	int BoundaryFlag = 1;
    long int EndlessLoop_num = 0;
	double thetaRand = 0;
	double RadiusTemp = 0;
	double Dij = 0;

	FILE *fp;
	//struct tm *newtime;
	char outfile[11]="UAV000.xls";
	//time_t t1;
	

	srand((unsigned)time(NULL));
	//**********************************************************//
	for(UAV_TotalNum=1; UAV_TotalNum<=Total_UAVsNum; UAV_TotalNum++)
	{
		for(i=0; i<Total_testcount; i++)
		{
			for(j=0; j<Total_TargetNum; j++)
				ScouterData1[i][j] = 0;
		}

		for(TargetNum=1; TargetNum<=Total_TargetNum; TargetNum++)
		{
			TestNum = 0;				// 测试计数
			for(test_count=1; test_count<=Total_testcount; test_count++)
			{
				printf("UAV_TotalNum=%d\t TargetNum=%d\t test_count=%d\n", UAV_TotalNum, TargetNum, test_count);

				//Sleep(1100);			//延时1.5秒
				CollisionNum = 0;		// 避障计数
				//**************************************************//
				// 随机生成目标点
				//TargetNum = 1;			// 目标总数
				
				AnnulusRandPoints(0,0,0.8*xMax,xMax,TargetNum);
				//**************************************************//
				for(i=0;i<=Total_UAVsNum;i++)
				{
					for(j=0;j<=Total_UAVsNum;j++)
						mu[i][j] = 1;
				}
				vi = vmax;						// 航行速度m/s
				m = floor(Tmax/delta_t);
				delta_Ri = vi * delta_t;		// delta_t秒内航行距离
				d = 2*vmax*delta_t + dmin;		// 安全航距
				
				Scouter_PosInitial(0,0,d,UAV_TotalNum);
				for(i=0;i<UAV_TotalNum;i++)
				{
					Last_ScouterPos[0][i] = ScouterPos[0][i];
					Last_ScouterPos[1][i] = ScouterPos[1][i];
					Last_ScouterPos[2][i] = ScouterPos[2][i];
				}
				//*************************************************//
				// 探测目标
				for(i=0;i<TargetNum;i++)
				{
					Detect_TargetPos[0][i] = 0;	// 存储探测到的目标坐标
					Detect_TargetPos[1][i] = 0;	
				}
				SimilarFlag = 0;
				TargetCount = 0;
				
				for(i=0;i<UAV_TotalNum;i++)
					Theta_1s[i] = 0;

				Xtemp = 0;
				Ytemp = 0;
				for(n=1;n<=nTotalNum;n++)
				{
					// 计算各无人机航行坐标
					for(i=0;i<UAV_TotalNum;i++)
					{
						if((n%m)==0)
						{
							BoundaryFlag = 1;
							EndlessLoop_num = 0;
							while(BoundaryFlag)
							{
								if(prod(&mu[i][0],UAV_TotalNum)==1)
									ScouterPos[2][i] = 2 * pi * rand()/(double)RAND_MAX; 
								else
								{
									ScouterPos[2][i] = Last_ScouterPos[2][i] + pi;
									BoundaryFlag = 0;
								}
								
								// 边界限制(只有间隔1s才能得到坐标)
								Xtemp = Last_ScouterPos[0][i] + vi * Tmax * cos(ScouterPos[2][i]);
								Ytemp = Last_ScouterPos[1][i] + vi * Tmax * sin(ScouterPos[2][i]);
								if(sqrt(pow(Xtemp,2) + pow(Ytemp,2)) <= xMax)
									BoundaryFlag = 0;
								else
								{
									// 防止死循环
									if(EndlessLoop_num>=500)
									{
										BoundaryFlag = 0;
										EndlessLoop_num = 0;
										if(Last_ScouterPos[1][i]<=0)
											ScouterPos[2][i] = acos(-Last_ScouterPos[0][i]/sqrt(pow(Last_ScouterPos[0][i],2)+pow(Last_ScouterPos[1][i],2)));
										else
											ScouterPos[2][i] = 2*pi - acos(-Last_ScouterPos[0][i]/sqrt(pow(Last_ScouterPos[0][i],2)+pow(Last_ScouterPos[1][i],2)));
									}
									else
									{
										EndlessLoop_num = EndlessLoop_num + 1;
									}
									Xtemp = Last_ScouterPos[0][i] + vi * Tmax * cos(ScouterPos[2][i]);
									Ytemp = Last_ScouterPos[1][i] + vi * Tmax * sin(ScouterPos[2][i]);
								}
								Theta_1s[i] = ScouterPos[2][i];
							}
						}
						else
						{
							if(prod(&mu[i][0],UAV_TotalNum)==1)      //无人机i与其他无人机间总冲突状态 
							{
								ScouterPos[2][i] = Last_ScouterPos[2][i];
								if(sqrt(pow(Xtemp,2) + pow(Ytemp,2)) > xMax)
									ScouterPos[2][i] = Theta_1s[i];
							}
							else
							{
								ScouterPos[2][i] = Last_ScouterPos[2][i] + pi;
							}
						}
						ScouterPos[2][i] = fmod(ScouterPos[2][i], 2*pi);
			
						ScouterPos[0][i] = Last_ScouterPos[0][i] + vi * delta_t * cos(ScouterPos[2][i]);  
						ScouterPos[1][i] = Last_ScouterPos[1][i] + vi * delta_t * sin(ScouterPos[2][i]);
						//*****************************************************************//
						// 探测目标
						for(k=0;k<TargetNum;k++)
						{
							RadiusTemp = sqrt(pow(ScouterPos[0][i]-TargetPos[0][k],2) + pow(ScouterPos[1][i]-TargetPos[1][k],2));
							if(RadiusTemp <= DetectRadius)
							{
								if(TargetCount > 0)
								{
									for(j=0;j<TargetCount;j++)
									{
										if((Detect_TargetPos[0][j]==TargetPos[0][k])&&(Detect_TargetPos[1][j]==TargetPos[1][k]))
										{
											SimilarFlag = 1;
											break;
										}
									}
									if(SimilarFlag==0)
									{
										Detect_TargetPos[0][TargetCount] = TargetPos[0][k];
										Detect_TargetPos[1][TargetCount] = TargetPos[1][k];
										TargetCount = TargetCount + 1;
									}
									SimilarFlag = 0;
								}
								else
								{
									Detect_TargetPos[0][TargetCount] = TargetPos[0][k];
									Detect_TargetPos[1][TargetCount] = TargetPos[1][k];
									TargetCount = TargetCount + 1;
								}
							}
						}
					}
					if((TargetCount >= TargetNum)||(n>=nTotalNum))
					{
						ScouterData1[TestNum][TargetNum-1] = n; 
						TestNum = TestNum + 1;
						break;
					}
					
					for(i=0;i<UAV_TotalNum;i++)
					{
						Last_ScouterPos[0][i] = ScouterPos[0][i];
						Last_ScouterPos[1][i] = ScouterPos[1][i];
						Last_ScouterPos[2][i] = ScouterPos[2][i];
					}
					//*********************************************************************//
					// 计算各无人机间冲突状态
					for(i=0;i<UAV_TotalNum;i++) 
					{
						for(j=0;j<UAV_TotalNum;j++)
						{
							if(j==i)
							{
								Dij = 0;
								mu[i][j] = 1;
							}
							else
							{
								Dij = sqrt(pow(ScouterPos[0][i]-ScouterPos[0][j],2) + pow(ScouterPos[1][i]-ScouterPos[1][j],2));
								if(Dij > d)
									mu[i][j] = 1;
								else
									mu[i][j] = 0;
							}
						} 
					}
				}
			}
		}
	//t1 = time(NULL);
	//newtime=localtime(&t1);
	//strftime(outfile, 128, "UAV000_%Y%m%d%H%M%S.xls", newtime);
	
	TempVar = UAV_TotalNum/100;
	outfile[3] = TempVar + 0x30;
	
	TempVar = (UAV_TotalNum - TempVar*100)/10;
	outfile[4] = TempVar + 0x30;
	TempVar = UAV_TotalNum%10;
	outfile[5] = TempVar + 0x30;

	fp = fopen(outfile,"w") ; 
	for(i=0; i<Total_testcount; i++)
	{
		for(j=0; j<Total_TargetNum; j++)
		{
			if(j<(Total_TargetNum-1))
				fprintf(fp,"%d\t",ScouterData1[i][j]);
			else
				fprintf(fp,"%d\n",ScouterData1[i][j]);
		}
	}
	fclose(fp);
	}
}

//==================================================================//
// AnnulusRandPoints：圆环内随机点生成函数
// x0,y0：圆心横纵坐标；Rmin,Rmax：圆环内径、外径；num_Dian:点的数量
//=================================================================//
void AnnulusRandPoints(double x0, double y0, double Rmin, double Rmax, int num_Dian)
{
	double r;
	double seta;
	int i;

	//srand((unsigned)time(NULL));

	for(i=0;i<num_Dian;i++)	
	{
		r = Rmin + (Rmax-Rmin)*rand()/(double)RAND_MAX; 
		seta = 2 * pi * rand()/(double)RAND_MAX;			// 得到生成点的角度，并利用极坐标形式画出点 	
		TargetPos[0][i] = x0 + r * cos(seta);	// 得到点的坐标 
		TargetPos[1][i] = y0 + r * sin(seta);
	}
}
//==================================================================//
// Scouter_PosInitial: 侦察者无人机初始位置生成函数
// 输出变量x,y,theta:侦察者无人机x坐标、y坐标和航向
// 输入变量x0,y0：圆心坐标；d:安全航距; Num:数量
//=================================================================//
void Scouter_PosInitial(double x0, double y0, double d, int Num)
{
	int i,j;
	int Flag;
	double r;
	double dij;
	double seta;

	//srand((unsigned)time(NULL));
		
	i = 0;
	Flag = 0;
	while(1)
	{
		r = 30 * rand()/(double)RAND_MAX; 
		seta = 2 * pi * rand()/(double)RAND_MAX; 
		// 得到点的坐标---随机扩散方程
		ScouterPos[0][i] = x0 + r * cos(seta);  		// 得到点的坐标 
		ScouterPos[1][i] = y0 + r * sin(seta);
		ScouterPos[2][i] = seta;
		
		if(i>0)
		{
			for(j=0;j<=i-1;j++)
			{
				dij = sqrt(pow(ScouterPos[0][i]-ScouterPos[0][j], 2) + pow(ScouterPos[1][i]-ScouterPos[1][j], 2));
				if(dij<=d)
					Flag = 1;
			}
		}
		//--------------------//
		if(Flag==0)
		{
			if(i >= Num-1)
				break;
			else
				i = i + 1;
		}
		Flag = 0;
	}
}
//==================================================================//
// 冲突状态累乘结果
// nu：冲突状态, num:无人机数量
//=================================================================//
char prod(int *un,int num)
{
	int i;
	
	for(i=0;i<num;i++)
	{
		if(*(un+i)==0)
			return 0;
	}
	return 1;
}
