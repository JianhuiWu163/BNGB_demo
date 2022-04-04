#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>
#include <string.h>

#define	pi					3.141592653
#define xMin				-200
#define xMax				200
#define yMin				-200
#define yMax				200
#define vmax				20				// 航行最大速度m/s

#define Total_testcount		1000
#define Total_TargetNum		20
#define Total_UAVsNum		100
#define nTotalNum			300000000


long int TestNum = 0;
long int test_count = 0;
long int CollisionNum = 0;

// 航向随机偏置算法
double lambda=0.997;					// 权重因子
int UAV_TotalNum=0;						// 无人机总数
int mu[Total_UAVsNum][Total_UAVsNum];	// 各无人机间冲突状态：1安全，0小于安全航距
//int last_mu[200][200];

double vi;								// 航行速度m/s
double Tmax = 1;						// 导航定位响应时间s
double delta_t = 0.01;					// 防冲突传感器响应时间s
int m;
double delta_Ri;						// delta_t秒内航行距离
double dmin = 1;						// 最小航行安全航距
double d;								// 安全航距
double DetectRadius = 20;				// 探测半径

double ScouterPos[3][Total_UAVsNum];			// 侦察者无人机的x坐标、y坐标和航向
double Last_ScouterPos[3][Total_UAVsNum];
//******************************************************************************//
// 随机生成目标点
long int TargetNum = 0;							// 目标总数
double TargetPos[2][Total_TargetNum];			// 目标x坐标、y坐标

// 探测目标
double Detect_TargetPos[2][Total_TargetNum];	// 存储探测到的目标坐标
long int SimilarFlag = 0;
long int TargetCount = 0;

double Theta_1s[Total_UAVsNum];

//long int ScouterData1[Total_testcount];
double ScouterData2[150000];
//******************************************************************************//
void AnnulusRandPoints(double x0, double y0, double Rmin, double Rmax, int num_Dian);
char prod(int *un,int num);
void Scouter_PosInitial(double x0, double y0, double d, int Num);
