#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <windows.h>
#include <string.h>

#define inf					0xffffffff
#define	pi					3.141592653
#define xMin				-200
#define xMax				200
#define yMin				-200
#define yMax				200
#define vmax				20				// ��������ٶ�m/s

#define Total_testcount		1000
#define Total_TargetNum		1
#define Total_UAVsNum		100
#define nTotalNum			30000000

#define ObsType				2	// 0��͹��Բ����1������Բ����2��ֱ��

#if ObsType==0
	#define startj	281
	#define endj	1281
	#define SONum	948
#elif ObsType==1
	#define startj	281
	#define endj	1281
	#define SONum	1782
#elif ObsType==2
	#define startj	281
	#define endj	1281
	#define SONum	2282
#endif

int smu[Total_UAVsNum][Total_TargetNum][SONum];

long int TestNum = 0;
long int CollisionNum = 0;


int UAV_TotalNum=0;						// ���˻�����
int mu[Total_UAVsNum][Total_UAVsNum];	// �����˻����ͻ״̬��1��ȫ��0С�ڰ�ȫ����

double vi;								// �����ٶ�m/s
double Tmax = 1;						// ������λ��Ӧʱ��s
double delta_t = 0.01;					// ����ͻ��������Ӧʱ��s
int mdelT;
double delta_Ri;						// delta_t���ں��о���
double dmin = 1;						// ��С���а�ȫ����
double d;								// ��ȫ����
double DetectRadius = 20;				// ̽��뾶

double ScouterPos[3][Total_UAVsNum];			// ��������˻���x���ꡢy����ͺ���
double Last_ScouterPos[3][Total_UAVsNum];
double BeforePos[3][Total_UAVsNum]; 
//******************************************************************************//
// �������Ŀ���
long int TargetNum = 0;							// Ŀ������
double TargetPos[2][Total_TargetNum];			// Ŀ��x���ꡢy����

// ̽��Ŀ��
double Detect_TargetPos[2][Total_TargetNum];	// �洢̽�⵽��Ŀ������
long int SimilarFlag = 0;
long int TargetCount = 0;

double Theta_1s[Total_UAVsNum];
long int ContinumFlag[Total_UAVsNum];
int infFlag[Total_UAVsNum];

int KeepFlag[Total_UAVsNum];
int FinishFlag[Total_UAVsNum];

long int DataBuff1[Total_testcount];
double DataBuff2[Total_testcount];


int OffsetDire[Total_UAVsNum];  // ������ñ�ײ����
double ObsAngle[Total_UAVsNum][SONum];
double ObsBuffer[2][Total_UAVsNum][SONum];
long int ObsBuferfNum[Total_UAVsNum]; 
//******************************************************************************//
void AnnulusRandPoints(double x0, double y0, double Rmin, double Rmax, int num_Dian);
char prod(int *un,int num);
char prod3(int *sun,int ly,int lz);
void Scouter_PosInitial(double x0, double y0, double d, int Num);
int sum(int *un, int len);
