#include "main.h"


//===========================================================//
void main()
{
	int TempVar=0;	
	int BoundaryFlag = 1;
	int TarObsLineFlag = 0;
	int FlagState[Total_UAVsNum];
	int RepeatFlag = 0;
	double Temp=0;
	double BiasAngle=0;
	double MaxObsAngle=0;
	double MinObsAngle=0;

	long int i,j,k,n;
	long int iters;
	long int CollisionNum;
	long int StaticObstacleNumber=0;

	//long int DataBuff1[Total_testcount];
	//double DataBuff2[Total_testcount];
	long int EndlessLoop_num = 0;

	double Xtemp = 0;
	double Ytemp = 0;
	double DijBefore = 0;
	double Dis[Total_UAVsNum];
	double last_Dis = 0;
	double Dobs = 0;
	double Dij = 0;
	double Radius = 0;
	double minAngle;
	double maxAngle;
	double TargetAngle;
	double phi;
	double StaticObstaclePos[2][Total_TargetNum][SONum];
	int StaticObstacleFlag;

	double Centerangle[Total_TargetNum];

	int zhinum;

	FILE *fp1,*fp2;
	char Filename1[18]="Iters_UAV001.xls";
	char Filename2[18]="Dij_UAV001.xls";

	srand((unsigned)time(NULL));
	
	//============================================================//  
	for(UAV_TotalNum=21; UAV_TotalNum<=50; UAV_TotalNum++)
	//for(UAV_TotalNum=Total_UAVsNum; UAV_TotalNum>=1; UAV_TotalNum--)
	{
		TestNum=0;		
		while(TestNum<Total_testcount)
		{
			printf("UAV_TotalNum=%d\t TestNum=%d\n", UAV_TotalNum, TestNum);
			
			CollisionNum = 0;
			DijBefore = inf;
			for(i=0;i<Total_UAVsNum;i++)
			{
				for(j=0;j<Total_UAVsNum;j++)
					mu[i][j] = 1;
			}
			for(i=0;i<Total_UAVsNum;i++)
			{
				for(j=0;j<Total_TargetNum;j++)
				{
					for(k=0;k<SONum;k++)
						smu[i][j][k] = 1;
				}
			}
			
			//=====================================//
			vi = vmax;						// �����ٶ�m/s
			mdelT = floor(Tmax/delta_t);
			delta_Ri = vi * delta_t;		// delta_t���ں��о���
			d = 2*vmax*delta_t + dmin;		// ��ȫ����

			// �������Ŀ���
			TargetNum = 1;	// Ŀ����		
			//AnnulusRandPoints(0,0,0.82*xMax,0.87*xMax,TargetNum);
			TargetPos[0][0] = 0.85 * xMax * cos(1*pi/4); 
			TargetPos[1][0]= 0.85 * xMax * sin(1*pi/4);
			// ���������������
			Scouter_PosInitial(0,0,d,UAV_TotalNum);
			// ���˻���Ŀ���Ƕ�
			for(i=0;i<UAV_TotalNum;i++)
			{
				Dis[i] = sqrt(pow(TargetPos[0][0]-ScouterPos[0][i],2) + pow(TargetPos[1][0]-ScouterPos[1][i],2));
				if(Dis[i]>0)
				{
					if((TargetPos[1][0]-ScouterPos[1][i])>=0)
						ScouterPos[2][i] = acos((TargetPos[0][0]-ScouterPos[0][i])/Dis[i]);
					else
						ScouterPos[2][i] = 2*pi - acos((TargetPos[0][0]-ScouterPos[0][i])/Dis[i]);
				}
				else
					ScouterPos[2][i] = 0;
			}

			for(i=0;i<UAV_TotalNum;i++)
			{
				Last_ScouterPos[0][i] = ScouterPos[0][i];
				Last_ScouterPos[1][i] = ScouterPos[1][i];
				Last_ScouterPos[2][i] = ScouterPos[2][i];
			}
			//============================================================//  
			// ����Ŀ�긽����̬�ϰ���
			for(i=0;i<TargetNum;i++)
			{
				Radius = sqrt(pow(TargetPos[0][i],2) + pow(TargetPos[1][i],2));
				if(-TargetPos[1][i]>=0) 
					Centerangle[i] = acos(-TargetPos[0][i]/Radius);
				else
					Centerangle[i] = 2*pi - acos(-TargetPos[0][i]/Radius);
				
				minAngle = Centerangle[i] - pi*70/180;
				maxAngle = Centerangle[i] + pi*70/180;
				for(j=0;j<startj;j++)
				{
					phi = j * pi/360;
					phi = phi + minAngle;
					StaticObstaclePos[0][i][j] = TargetPos[0][i] + 50 * cos(phi); 
					StaticObstaclePos[1][i][j] = TargetPos[1][i] + 50 * sin(phi); 
				}
			}
			//============================================================//  
			if(ObsType==0)			// 0��͹��Բ����1������Բ����2��ֱ��
			{
				for(i=0;i<TargetNum;i++)
				{
					minAngle = Centerangle[0] - pi*60/180;
					maxAngle = Centerangle[0] + pi*60/180;
					for(j=0;j<667;j++)
					{
						phi = j * pi/1000;
						phi = phi + minAngle;
						StaticObstaclePos[0][i][j+startj] = TargetPos[0][i] + 110 * cos(phi); 
						StaticObstaclePos[1][i][j+startj] = TargetPos[1][i] + 110 * sin(phi); 
					}
				}
			}
			else if(ObsType==1)
			{
				for(i=0;i<TargetNum;i++)
				{
					minAngle = Centerangle[0] + pi - pi*90/180;
					maxAngle = Centerangle[0] + pi + pi*180/180;
					for(j=0;j<1501;j++)
					{
						phi = j * pi/1000;
						phi = phi + minAngle;
						StaticObstaclePos[0][i][j+startj] = 110 * cos(phi); 
						StaticObstaclePos[1][i][j+startj] = 110 * sin(phi); 
					}
				}
			}
			else if(ObsType==2)
			{
				for(i=0;i<TargetNum;i++)
				{
					minAngle = fmod(Centerangle[0] + pi + 90*pi/180,2*pi);
					Xtemp = 80 * cos(Centerangle[0] + pi); 
					Ytemp = 80 * sin(Centerangle[0] + pi); 

					j = startj;
					for(zhinum=500;zhinum>=1;zhinum--)
					{
						StaticObstaclePos[0][i][j] = Xtemp + zhinum * cos(minAngle) * 0.2; 
						StaticObstaclePos[1][i][j] = Ytemp + zhinum * sin(minAngle) * 0.2; 
						j = j + 1;
					}
					StaticObstaclePos[0][i][j] = Xtemp; 
					StaticObstaclePos[1][i][j] = Ytemp; 
					j = j + 1;

					for(zhinum=1;zhinum<=500;zhinum++) 
					{
						StaticObstaclePos[0][i][j] = Xtemp -  zhinum * cos(minAngle) * 0.2; 
						StaticObstaclePos[1][i][j] = Ytemp -  zhinum * sin(minAngle) * 0.2; 
						j = j + 1;
					}
				}
				//==��U���ϰ���==//
				for(i=0;i<TargetNum;i++)
				{
					minAngle = fmod(Centerangle[0] + pi,2*pi);
					Xtemp = StaticObstaclePos[0][i][startj];
					Ytemp = StaticObstaclePos[1][i][startj];
					for(zhinum=1;zhinum<=500;zhinum++)
					{
						StaticObstaclePos[0][i][j] = Xtemp - zhinum * cos(minAngle) * 0.2; 
						StaticObstaclePos[1][i][j] = Ytemp - zhinum * sin(minAngle) * 0.2; 
						j = j + 1;
					}
					minAngle = fmod(Centerangle[0] + pi,2*pi);
					Xtemp = StaticObstaclePos[0][i][endj];
					Ytemp = StaticObstaclePos[1][i][endj];
					for(zhinum=1;zhinum<=500;zhinum++)
					{
						StaticObstaclePos[0][i][j] = Xtemp -  zhinum * cos(minAngle) * 0.2; 
						StaticObstaclePos[1][i][j] = Ytemp -  zhinum * sin(minAngle) * 0.2; 
						j = j + 1;
					}
				}
			}
			
			for(i=0;i<Total_UAVsNum;i++)
			{
				for(j=0;j<SONum;j++)
				{
					ObsAngle[i][j] = 0;
					ObsBuffer[0][i][j] = 0;
					ObsBuffer[1][i][j] = 0;
				}
				ObsBuferfNum[i] = 0;
				Theta_1s[i] = 0;
				BeforePos[0][i] = 0;
				BeforePos[1][i] = 0;
				BeforePos[2][i] = 0;
				ContinumFlag[i] = 0;
				infFlag[i] = 0;
				KeepFlag[i] = 0;
				FinishFlag[i] = 0;
				FlagState[i] = 0;
			}
			
			Xtemp = 0;
			Ytemp = 0;
			BiasAngle = 0;
			// ˳ʱ�롢��ʱ�뷽�����ѡ��
			//OffsetDire[0] = 1;
			//OffsetDire[1] = -1;
			for(i=0;i<UAV_TotalNum;i++)
			{
				if(rand()/(double)RAND_MAX < 0.5)
					OffsetDire[i] = -1;
				else
					OffsetDire[i] = 1;
			}
			//===================================================//
			for(iters=1;iters<=nTotalNum;iters++)
			{	
				if(iters>=10000)
					break;

				if(sum(&FinishFlag[0],Total_UAVsNum)>=UAV_TotalNum)
				{
					DataBuff1[TestNum] = iters; 
					DataBuff2[TestNum] = DijBefore;
					TestNum = TestNum + 1;
					break;
				}
				for(i=0;i<UAV_TotalNum;i++)
				{
					if(FinishFlag[i]!=1)
					{
						if(iters % mdelT==0)
						{
							BoundaryFlag = 1;
							EndlessLoop_num = 0;
							while(BoundaryFlag)
							{
								last_Dis = Dis[i];
								Dis[i] = sqrt(pow(TargetPos[0][0]-ScouterPos[0][i],2) + pow(TargetPos[1][0]-ScouterPos[1][i],2));
								if((prod(&mu[i][0],UAV_TotalNum)==1)&&(prod3(&smu[i][0][0],Total_TargetNum,SONum)==1))
								{
									// ���˻���Ŀ���Ƕ�
									if(Dis[i]>0)
									{
										if((TargetPos[1][0]-ScouterPos[1][i])>=0)
											TargetAngle = acos((TargetPos[0][0]-ScouterPos[0][i])/Dis[i]);
										else
											TargetAngle = 2*pi - acos((TargetPos[0][0]-ScouterPos[0][i])/Dis[i]);
									}
									else
									{
										TargetAngle = 0;
									}
									
									StaticObstacleNumber = 0;
									StaticObstacleFlag = 0;
									for(n=0;n<SONum;n++)
										ObsAngle[i][n] = 0;
									
									TarObsLineFlag = 0;
									for(n=0;n<SONum;n++)
									{
										Dobs = sqrt(pow(ScouterPos[0][i]-StaticObstaclePos[0][0][n],2) + pow(ScouterPos[1][i]-StaticObstaclePos[1][0][n],2));
										if(Dobs <= DetectRadius)
										{
											StaticObstacleFlag = 1;
											if(Dobs>0)
											{
												if((StaticObstaclePos[1][0][n]-ScouterPos[1][i])>=0)
													ObsAngle[i][StaticObstacleNumber] = acos((StaticObstaclePos[0][0][n]-ScouterPos[0][i])/Dobs);
												else
													ObsAngle[i][StaticObstacleNumber] = 2*pi - acos((StaticObstaclePos[0][0][n]-ScouterPos[0][i])/Dobs);
											}
											else
											{
												ObsAngle[i][StaticObstacleNumber] = 0;
											}
											// ���ӷ�Χ�ڣ��ж��ϰ����Ƿ��谭���˻�����Ŀ���
											if(((fabs(TargetAngle - ObsAngle[i][StaticObstacleNumber])<= (5*pi/180))&&(Dobs<Dis[i]))||(Dis[i]>last_Dis))  // 5*pi/180�Ƿ�ֹ����ʱ�����õ��ϰ���㲻���ܼ�
												TarObsLineFlag = 1;			// ����TarObsLineFlag���ֵ�����
											
											// �洢��̽�⵽���ϰ���
											RepeatFlag = 0;
											if(ObsBuferfNum[i]>0)
											{
												for(j=0;j<ObsBuferfNum[i];j++)
												{
													if((ObsBuffer[0][i][j]==StaticObstaclePos[0][0][n])&&(ObsBuffer[1][i][j]==StaticObstaclePos[1][0][n]))
													{
														RepeatFlag = 1;		// �ж�����֪�ϰ���������ͬ�����洢 
														break;
													}
												}
											}
											if(RepeatFlag==0)		// �޳���ͬ���ϵ����꣬�Դ˼��ٴ洢�ռ�
											{
												ObsBuffer[0][i][ObsBuferfNum[i]] = StaticObstaclePos[0][0][n];	// ��֪�ϰ���
												ObsBuffer[1][i][ObsBuferfNum[i]] = StaticObstaclePos[1][0][n];
												ObsBuferfNum[i] = ObsBuferfNum[i]  + 1;
											}
											StaticObstacleNumber = StaticObstacleNumber + 1;
										}
									}
									// �ж���֪�ϰ����Ƿ��赲���˻���Ŀ���
									if((StaticObstacleFlag==1)&&(TarObsLineFlag!=1))
									{
										for(n=0;n<ObsBuferfNum[i];n++)
										{
											Dobs = sqrt(pow(ScouterPos[0][i]-ObsBuffer[0][i][n],2) + pow(ScouterPos[1][i]-ObsBuffer[1][i][n],2));
											if(Dobs>0)
											{
												if((ObsBuffer[1][i][n]-ScouterPos[1][i])>=0)
													Temp = acos((ObsBuffer[0][i][n]-ScouterPos[0][i])/Dobs);
												else
													Temp = 2*pi - acos((ObsBuffer[0][i][n]-ScouterPos[0][i])/Dobs);
											}
											else
											{
												Temp = 0;
											}
											// ���ӷ�Χ�ڣ��ж��ϰ����Ƿ��谭���˻�����Ŀ���
											if((fabs(TargetAngle - Temp)<= (5*pi/180))&&(Dobs<Dis[i]))  	// 5*pi/180�Ƿ�ֹ����ʱ�����õ��ϰ���㲻���ܼ�
											{	
												TarObsLineFlag = 1;                                     // ����TarObsLineFlag���ֵ�����
												break;
											}
										}
									}    
									//=====================================================================//
									if((StaticObstacleFlag==1)&&(TarObsLineFlag==1))
									{
										// ������任--y��ָ��Ŀ���
										if(TargetPos[1][0]>=0)
											BiasAngle = acos(TargetPos[0][0]/sqrt(pow(TargetPos[0][0],2)+pow(TargetPos[1][0],2)));
										else
											BiasAngle = 2*pi - acos(TargetPos[0][0]/sqrt(pow(TargetPos[0][0],2)+pow(TargetPos[1][0],2)));
										
										for(j=0;j<StaticObstacleNumber;j++)
										{
											//ObsAngle[i][j] = fmod(ObsAngle[i][j]-(BiasAngle - pi/2),2*pi);
											ObsAngle[i][j] = fmod(ObsAngle[i][j]-BiasAngle,2*pi);
											if(ObsAngle[i][j]<0)
												ObsAngle[i][j] = ObsAngle[i][j] + 2*pi;
										}
										// ð������--��С��������
										for(n=0;n<StaticObstacleNumber-1;n++)
										{
											for(j=0;j<StaticObstacleNumber-1-n;j++)
											{
												if(ObsAngle[i][j]>ObsAngle[i][j+1])
												{
													Temp = ObsAngle[i][j+1];
													ObsAngle[i][j+1] = ObsAngle[i][j];
													ObsAngle[i][j] = Temp;
												}
											}
										}
										// �������˻�����
										if(OffsetDire[i]==1)  // ��ʱ��
										{
											for(j=0;j<=StaticObstacleNumber-2;j++) 
											{
												if(fabs(ObsAngle[i][j] - ObsAngle[i][j+1])>(20*pi/180))    // ���Ƕ��Ƿ�����
												{
													//MaxObsAngle = ObsAngle[i][j] + (BiasAngle - pi/2);
													MaxObsAngle = ObsAngle[i][j] + BiasAngle;
													break;
												}
												if(j==(StaticObstacleNumber-2))
													//MaxObsAngle = ObsAngle[i][j+1] + (BiasAngle - pi/2);
													MaxObsAngle = ObsAngle[i][j+1] + BiasAngle;
											}
											ScouterPos[2][i] = MaxObsAngle + OffsetDire[i] * (pi/18 + (pi/6)*rand()/(double)RAND_MAX);
										}
										else if(OffsetDire[i]==-1) //˳ʱ��
										{
											if((ObsAngle[i][StaticObstacleNumber-1]>350*pi/180)&&(ObsAngle[i][0]<10*pi/180))
											{
												// ���Ƕ��Ƿ�����
												for(j=StaticObstacleNumber-1;j>=1;j--)
												{
													if(fabs(ObsAngle[i][j] - ObsAngle[i][j-1])>(20*pi/180))    // 0�ȸ����ڽ�����λƫ��ϴ�����ѡ��20*pi/180
													{
														//MinObsAngle = ObsAngle[i][j] + (BiasAngle - pi/2);
														MinObsAngle = ObsAngle[i][j] + BiasAngle;
														break;
													}
												}
											}
											else
											{
												//MinObsAngle = ObsAngle[i][0] + (BiasAngle - pi/2);
												MinObsAngle = ObsAngle[i][0] + BiasAngle;
											}
											ScouterPos[2][i] = MinObsAngle + OffsetDire[i] * (pi/18 + (pi/6)*rand()/(double)RAND_MAX); 
										}
										KeepFlag[i] = 0;
									}
									else
									{
										if(Dis[i]>=10)
											ScouterPos[2][i] = TargetAngle + OffsetDire[i] * (pi/4) * rand()/(double)RAND_MAX;    //����rand*pi/4��ֹ������
										else
										{
											ScouterPos[2][i] = TargetAngle;
											//����Ŀ�ĵ���ͣ��������
											if(Dis[i]<0.8)
												FinishFlag[i] = 1;
										}
									}

								}
								else
								{
									ScouterPos[2][i]= Last_ScouterPos[2][i] + pi;
									BoundaryFlag = 0;
								}
								// �߽�����(ֻ�м��1s���ܵõ�����)
								Xtemp = Last_ScouterPos[0][i] + vi * Tmax * cos(ScouterPos[2][i]);
								Ytemp = Last_ScouterPos[1][i] + vi * Tmax * sin(ScouterPos[2][i]);
								if(sqrt(pow(Xtemp,2) + pow(Ytemp,2)) <= xMax)
									BoundaryFlag = 0;
								else
								{
									// ��ֹ��ѭ��
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
									Theta_1s[i] = ScouterPos[2][i];
								}
							}
						}
						else
						{
							if((prod(&mu[i][0],UAV_TotalNum)==1)&&(prod3(&smu[i][0][0],Total_TargetNum,SONum)==1))
							{
								ScouterPos[2][i] = Last_ScouterPos[2][i];
								if(sqrt(pow(Xtemp,2)+pow(Ytemp,2)) > xMax)
									ScouterPos[2][i] = Theta_1s[i];
							}
							else
								ScouterPos[2][i] = Last_ScouterPos[2][i] + pi;

							// ����Ŀ�ĵ���ͣ��������
							if(sqrt(pow(TargetPos[0][0]-ScouterPos[0][i],2) + pow(TargetPos[1][0]-ScouterPos[1][i],2))<0.8)
								FinishFlag[i] = 1;
						}
						ScouterPos[2][i] = fmod(ScouterPos[2][i],2*pi);
						ScouterPos[0][i] = Last_ScouterPos[0][i] + vi * delta_t * cos(ScouterPos[2][i]);  
						ScouterPos[1][i] = Last_ScouterPos[1][i] + vi * delta_t * sin(ScouterPos[2][i]);
					}
				}
				////////////////////////////////////////////////
				for(i=0;i<UAV_TotalNum;i++)
				{
					Last_ScouterPos[0][i] = ScouterPos[0][i];
					Last_ScouterPos[1][i] = ScouterPos[1][i];
					Last_ScouterPos[2][i] = ScouterPos[2][i];
				}
				if(iters % mdelT==0)
				{
					for(i=0;i<UAV_TotalNum;i++)
					{
						BeforePos[0][i] = ScouterPos[0][i];
						BeforePos[1][i] = ScouterPos[1][i];
						BeforePos[2][i] = ScouterPos[2][i];
					}
				}
				//*********************************************************************//
				// ��������˻����ͻ״̬
				for(i=0;i<UAV_TotalNum;i++) 
				{
					for(j=0;j<UAV_TotalNum;j++)
					{
						if((FinishFlag[i]==1)||(FinishFlag[j]==1))
							mu[i][j] = 1;
						else
						{
							if(j==i)
							{
								Dij = 0;
								mu[i][j] = 1;
							}
							else
							{
								Dij = sqrt(pow(ScouterPos[0][i]-ScouterPos[0][j],2) + pow(ScouterPos[1][i]-ScouterPos[1][j],2));

								if(Dij < DijBefore)
									DijBefore = Dij;

								if(Dij > d)
									mu[i][j] = 1;
								else
								{
									mu[i][j] = 0;
									//CollisionNum = CollisionNum + 1;
									//DataBuff2[CollisionNum] = Dij; 
								}
							}
						}
					} 

					for(j=0;j<TargetNum;j++)
					{
						for(k=0;k<SONum;k++)
						{
							Dij = sqrt(pow(ScouterPos[0][i]-StaticObstaclePos[0][j][k],2) + pow(ScouterPos[1][i]-StaticObstaclePos[1][j][k],2));

							if(Dij < DijBefore)
								DijBefore = Dij;

							if((Dij > d)||(FinishFlag[i]==1))
								smu[i][j][k] = 1;
							else
							{
								smu[i][j][k] = 0;
								//CollisionNum = CollisionNum + 1;
								//DataBuff2[CollisionNum] = Dij; 
							}
						}
					}
				}
			}
		}
		
		TempVar = UAV_TotalNum/100;
		Filename1[9] = TempVar + 0x30;
		TempVar = (UAV_TotalNum%100)/10;
		Filename1[10] = TempVar + 0x30;
		TempVar = UAV_TotalNum%10;
		Filename1[11] = TempVar + 0x30;
		
		fp1 = fopen(Filename1,"w") ; 
		for(i=0; i<Total_testcount; i++)
		{
			fprintf(fp1,"%d\n",DataBuff1[i]);
		}
		fclose(fp1);
		
		Filename2[7] = Filename1[9];
		Filename2[8] = Filename1[10];
		Filename2[9] = Filename1[11];		
		fp2 = fopen(Filename2,"aw"); 
		for(i=0; i<Total_testcount; i++)
		{
			fprintf(fp2,"%lf\n",DataBuff2[i]);
		}
		fclose(fp2);
	}
}
//==================================================================//
// AnnulusRandPoints��Բ������������ɺ���
// x0,y0��Բ�ĺ������ꣻRmin,Rmax��Բ���ھ����⾶��num_Dian:�������
//=================================================================//
void AnnulusRandPoints(double x0, double y0, double Rmin, double Rmax, int num_Dian)
{
	double r;
	double seta;
	int i;

	for(i=0;i<num_Dian;i++)	
	{
		r = Rmin + (Rmax-Rmin)*rand()/(double)RAND_MAX; 
		seta = 2 * pi * rand()/(double)RAND_MAX;			// �õ����ɵ�ĽǶȣ������ü�������ʽ������ 	
		TargetPos[0][i] = x0 + r * cos(seta);	// �õ�������� 
		TargetPos[1][i] = y0 + r * sin(seta);
	}
}
//==================================================================//
// Scouter_PosInitial: ��������˻���ʼλ�����ɺ���
// �������x,y,theta:��������˻�x���ꡢy����ͺ���
// �������x0,y0��Բ�����ꣻd:��ȫ����; Num:����
//=================================================================//
void Scouter_PosInitial(double x0, double y0, double d, int Num)
{
	int i,j;
	int Flag;
	double r;
	double dij;
	double seta;
		
	i = 0;
	Flag = 0;
	while(1)
	{
		r = 50 * rand()/(double)RAND_MAX; 
		seta = 2 * pi * rand()/(double)RAND_MAX; 
		// �õ��������---�����ɢ����
		ScouterPos[0][i] = x0 + r * cos(seta);  		// �õ�������� 
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
// ��ͻ״̬�۳˽��
// nu����ͻ״̬, num:���˻�����
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

//==================================================================//
// ��ͻ״̬�۳˽��
// snu����ͻ״̬, ly,lz:���鳤��
//=================================================================//
char prod3(int *sun,int ly,int lz)
{
	int i,j;
	//int temp;
	for(i=0;i<ly;i++)
	{
		for(j=0;j<lz;j++)
		{
			//temp = *(sun+i*lz+j);
			if(*(sun+i*lz+j)==0)
				return 0;
		}
	}
	return 1;
}
//==================================================================//
// �ۼ�
// nu������
//=================================================================//
int sum(int *un, int len)
{
	int i;
	int sum=0;
	
	for(i=0;i<len;i++)
	{
		sum += *(un+i);
	}
	return sum;
}