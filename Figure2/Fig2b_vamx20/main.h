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
#define vmax				20				// ��������ٶ�m/s

#define Total_testcount		1000
#define Total_TargetNum		20
#define Total_UAVsNum		100
#define nTotalNum			300000000


long int TestNum = 0;
long int test_count = 0;
long int CollisionNum = 0;

// �������ƫ���㷨
double lambda=0.997;					// Ȩ������
int UAV_TotalNum=0;						// ���˻�����
int mu[Total_UAVsNum][Total_UAVsNum];	// �����˻����ͻ״̬��1��ȫ��0С�ڰ�ȫ����
//int last_mu[200][200];

double vi;								// �����ٶ�m/s
double Tmax = 1;						// ������λ��Ӧʱ��s
double delta_t = 0.01;					// ����ͻ��������Ӧʱ��s
int m;
double delta_Ri;						// delta_t���ں��о���
double dmin = 1;						// ��С���а�ȫ����
double d;								// ��ȫ����
double DetectRadius = 20;				// ̽��뾶

double ScouterPos[3][Total_UAVsNum];			// ��������˻���x���ꡢy����ͺ���
double Last_ScouterPos[3][Total_UAVsNum];
//******************************************************************************//
// �������Ŀ���
long int TargetNum = 0;							// Ŀ������
double TargetPos[2][Total_TargetNum];			// Ŀ��x���ꡢy����

// ̽��Ŀ��
double Detect_TargetPos[2][Total_TargetNum];	// �洢̽�⵽��Ŀ������
long int SimilarFlag = 0;
long int TargetCount = 0;

double Theta_1s[Total_UAVsNum];

//long int ScouterData1[Total_testcount];
double ScouterData2[150000];
//******************************************************************************//
void AnnulusRandPoints(double x0, double y0, double Rmin, double Rmax, int num_Dian);
char prod(int *un,int num);
void Scouter_PosInitial(double x0, double y0, double d, int Num);
