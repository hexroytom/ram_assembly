// pole_det.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <math.h>
#include <memory.h>
#include <iostream>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>

#define DetPoleNum 5
#define CliffDis 100
#define DETECT_POLE_NUM_MAX 5
#define StepLen 2
#define CheckRowN 50
#define Square -1
#define Round 1

using namespace std;
using namespace cv;

typedef struct poleInfoDef   
{
	char PoleStyle;   //�˼����ͣ�1��ʾԲ�ˣ�-1��ʾ����
	float PoleFunc[2][3];  //�˼����߷���
	float PoleDiameter;  //�˼�ֱ��
	unsigned int PoleLenght; //�˳�
	float Pose[3];   //�˼�������̬
	float BendDeg;   //�˼�������
	float Cylinder_Dis; //�˼���������Բ����֮��ļ�϶ֵ
}PoleInfo;

float P2Distance(const float* P1,const float* P2)
{
	float p2distance;
	p2distance = sqrt((P1[0]-P2[0])*(P1[0]-P2[0])+(P1[1]-P2[1])*(P1[1]-P2[1])+(P1[2]-P2[2])*(P1[2]-P2[2]));
	return p2distance;
}

/*void BoundaryLineFit(int LinePN, float* LineP, float* LineDir, float* OnLineP)
���ܣ�������������㷨
���룺LinePN ���ص�������LineP ������������
�����LineDir �������ߵķ���������OnLineP �������߶ε��е�
���أ�
*/
void BoundaryLineFit(int LinePN, float* LineP, float* LineDir, float* OnLineP)//������������㷨��HU����P24
{
	int i,j;
	float FitPs[640][3];
	//	float poleline[3];
	for (int i=0; i<LinePN; i++)
	{
		for (int j=0; j<3; j++)
		{
			FitPs[i][j] = *(LineP++);
		}
	}
	//������
	double sum_xy = 0;
	double sum_yy = 0;
	double sum_zy = 0;
	//һ����
	double sum_x = 0;
	double sum_y = 0;
	double sum_z = 0;
	for (i=0;i<LinePN;i++)
	{
		sum_xy += FitPs[i][0]*FitPs[i][1];
		sum_yy += FitPs[i][1]*FitPs[i][1];
		sum_zy += FitPs[i][2]*FitPs[i][1];
		sum_x  += FitPs[i][0];
		sum_y  += FitPs[i][1];
		sum_z  += FitPs[i][2];
	}
	//��ֱ�ߵķ���������һ��
	//��������LineDir
	LineDir[0] = (LinePN*sum_xy-sum_x*sum_y)/(LinePN*sum_yy-sum_y*sum_y);
	LineDir[1] = 1;
	LineDir[2] = (LinePN*sum_zy-sum_z*sum_y)/(LinePN*sum_yy-sum_y*sum_y);
	//ֱ����һ��OnLineP
	// 	OnLineP[0] = (sum_x-LineDir[0]*sum_y)/LinePN;
	// 	OnLineP[1] = 0;
	// 	OnLineP[2] = (sum_z-LineDir[2]*sum_y)/LinePN;
	OnLineP[0] = sum_x/LinePN;
	OnLineP[1] = sum_y/LinePN;
	OnLineP[2] = sum_z/LinePN;
	//�޳������
	double PtoLineD;
	double PtoP[3];
	double DeletePN[1000];
	int deletep = 0;
	for (i=0;i<LinePN;i++)
	{
		PtoP[0] = FitPs[i][0] - OnLineP[0];
		PtoP[1] = FitPs[i][1] - OnLineP[1];
		PtoP[2] = FitPs[i][2] - OnLineP[2];
		PtoLineD = (sqrt((LineDir[1]*PtoP[2]-LineDir[2]*PtoP[1])*(LineDir[1]*PtoP[2]-LineDir[2]*PtoP[1]) 
			+(LineDir[2]*PtoP[0]-LineDir[0]*PtoP[2])*(LineDir[2]*PtoP[0]-LineDir[0]*PtoP[2])
			+(LineDir[0]*PtoP[1]-LineDir[1]*PtoP[0])*(LineDir[0]*PtoP[1]-LineDir[1]*PtoP[0]))
			)/
			(sqrt(LineDir[0]*LineDir[0]+LineDir[1]*LineDir[1]+LineDir[2]*LineDir[2]));
		if (PtoLineD>=40)
		{
			DeletePN[deletep] = i;
			deletep++;
		}
	}
	DeletePN[deletep] = LinePN-1;
	//�޳����ֱ�߷���
	if(deletep != 0)
	{
		for (i=0;i<deletep;i++)
		{
			for (int k=DeletePN[i];k<DeletePN[i+1];k++)
			{
				for (int z=0; z<3; z++)
				{
					FitPs[k-i][z] = FitPs[k+1][z];
				}
			}
		}
		sum_xy = 0;
		sum_yy = 0;
		sum_zy = 0;
		//һ����
		sum_x = 0;
		sum_y = 0;
		sum_z = 0;
		for (i=0;i<LinePN-deletep-2;i++)
		{
			sum_xy += FitPs[i][0]*FitPs[i][1];
			sum_yy += FitPs[i][1]*FitPs[i][1];
			sum_zy += FitPs[i][2]*FitPs[i][1];
			sum_x  += FitPs[i][0];
			sum_y  += FitPs[i][1];
			sum_z  += FitPs[i][2];
		}
		//��ֱ�ߵķ���������һ��
		//��������LineDir
		LineDir[0] = (LinePN*sum_xy-sum_x*sum_y)/(LinePN*sum_yy-sum_y*sum_y);//���������в���
		LineDir[1] = 1;
		LineDir[2] = (LinePN*sum_zy-sum_z*sum_y)/(LinePN*sum_yy-sum_y*sum_y);
		//ֱ����һ��OnLineP
		// 		OnLineP[0] = (sum_x-LineDir[0]*sum_y)/LinePN;
		// 		OnLineP[1] = 0;
		// 		OnLineP[2] = (sum_z-LineDir[2]*sum_y)/LinePN;
		OnLineP[0] = sum_x/LinePN;
		OnLineP[1] = sum_y/LinePN;
		OnLineP[2] = sum_z/LinePN;
	}
}

// �㵽ֱ�ߵľ��뺯��
float PtoLineDis(float* Cal_P, float* LineDir, float* OnLineP)
{
	float PtoP[3],PtoLineD;
	PtoP[0] = Cal_P[0] - OnLineP[0];
	PtoP[1] = Cal_P[1] - OnLineP[1];
	PtoP[2] = Cal_P[2] - OnLineP[2];
	PtoLineD = (sqrt((LineDir[1]*PtoP[2]-LineDir[2]*PtoP[1])*(LineDir[1]*PtoP[2]-LineDir[2]*PtoP[1]) 
		+(LineDir[2]*PtoP[0]-LineDir[0]*PtoP[2])*(LineDir[2]*PtoP[0]-LineDir[0]*PtoP[2])
		+(LineDir[0]*PtoP[1]-LineDir[1]*PtoP[0])*(LineDir[0]*PtoP[1]-LineDir[1]*PtoP[0]))
		)/
		(sqrt(LineDir[0]*LineDir[0]+LineDir[1]*LineDir[1]+LineDir[2]*LineDir[2]));
	return PtoLineD;
}

// �㵽ƽ��ľ��� , ����������
float PtoPlaneDis(bool Abs, float* PlaneV, float* OnPlaneP, float* MeasureP)
{
	float PtoPlaneD;
	if (Abs == true)
    {
		PtoPlaneD = abs((PlaneV[0]*(OnPlaneP[0]-MeasureP[0])+PlaneV[1]*(OnPlaneP[1]-MeasureP[1])+PlaneV[2]*(OnPlaneP[2]-MeasureP[2]))
			/(sqrt(PlaneV[0]*PlaneV[0]+PlaneV[1]*PlaneV[1]+PlaneV[2]*PlaneV[2])));
	}
	else
		PtoPlaneD = (PlaneV[0]*(OnPlaneP[0]-MeasureP[0])+PlaneV[1]*(OnPlaneP[1]-MeasureP[1])+PlaneV[2]*(OnPlaneP[2]-MeasureP[2]))
		/(sqrt(PlaneV[0]*PlaneV[0]+PlaneV[1]*PlaneV[1]+PlaneV[2]*PlaneV[2]));
	return PtoPlaneD;
}

// ���׾�������
bool MatrixN3Inv(float* MatrixN3, float* InvMatrixN3)
{
	float Det = MatrixN3[0]*MatrixN3[4]*MatrixN3[8]+MatrixN3[1]*MatrixN3[5]*MatrixN3[6]+
		MatrixN3[2]*MatrixN3[3]*MatrixN3[7]-MatrixN3[2]*MatrixN3[4]*MatrixN3[6]-
		MatrixN3[1]*MatrixN3[3]*MatrixN3[8]-MatrixN3[0]*MatrixN3[5]*MatrixN3[7];
	if (Det!=0)
	{
		InvMatrixN3[0] =  (MatrixN3[4]*MatrixN3[8]-MatrixN3[5]*MatrixN3[7])/Det;
		InvMatrixN3[3] = -(MatrixN3[3]*MatrixN3[8]-MatrixN3[5]*MatrixN3[6])/Det;
		InvMatrixN3[6] =  (MatrixN3[3]*MatrixN3[7]-MatrixN3[4]*MatrixN3[6])/Det;
		InvMatrixN3[1] = -(MatrixN3[1]*MatrixN3[8]-MatrixN3[2]*MatrixN3[7])/Det;
		InvMatrixN3[4] =  (MatrixN3[0]*MatrixN3[8]-MatrixN3[2]*MatrixN3[6])/Det;
		InvMatrixN3[7] = -(MatrixN3[0]*MatrixN3[7]-MatrixN3[1]*MatrixN3[6])/Det;
		InvMatrixN3[2] =  (MatrixN3[1]*MatrixN3[5]-MatrixN3[2]*MatrixN3[4])/Det;
		InvMatrixN3[5] = -(MatrixN3[0]*MatrixN3[5]-MatrixN3[2]*MatrixN3[3])/Det;
		InvMatrixN3[8] =  (MatrixN3[0]*MatrixN3[4]-MatrixN3[1]*MatrixN3[3])/Det;
		return true;
	}
	else
		return false;
}

// ������λ��
void VectorUnitization(float* Vector)
{
	float Vectorlength = sqrt(Vector[0]*Vector[0]+Vector[1]*Vector[1]+Vector[2]*Vector[2]);
	Vector[0] = Vector[0]/Vectorlength;
	Vector[1] = Vector[1]/Vectorlength;
	Vector[2] = Vector[2]/Vectorlength;
}

//Plane ������ĸ����ֱ��ǣ�A,B,C��,Ax+By+Cz+1=0
bool PlaneFitting(int FitPointNum, float* FitPoint, float* Plane)
{
	int i,j;
	float FitPs[10000][3];
	//	float poleline[3];
	for (int i=0; i<FitPointNum; i++)
	{
		for (int j=0; j<3; j++)
		{
			FitPs[i][j] = *(FitPoint++);
		}
	}
	//������
	double sum_xx = 0;
	double sum_yy = 0;
	double sum_zz = 0;
	double sum_xy = 0;
	double sum_xz = 0;
	double sum_yz = 0;
	//һ����
	double sum_x = 0;
	double sum_y = 0;
	double sum_z = 0;
	for (i=0;i<FitPointNum;i++)
	{
		sum_xx += FitPs[i][0]*FitPs[i][0];
		sum_yy += FitPs[i][1]*FitPs[i][1];
		sum_zz += FitPs[i][2]*FitPs[i][2];
		sum_xy += FitPs[i][0]*FitPs[i][1];
		sum_xz += FitPs[i][0]*FitPs[i][2];
		sum_yz += FitPs[i][1]*FitPs[i][2];
		sum_x  += FitPs[i][0];
		sum_y  += FitPs[i][1];
		sum_z  += FitPs[i][2];
	}
	float Mtx[9];
	float InvMtx[9];
	Mtx[0] = sum_xx; Mtx[1] = sum_xy; Mtx[2] = sum_xz;
	Mtx[3] = sum_xy; Mtx[4] = sum_yy; Mtx[5] = sum_yz;
	Mtx[6] = sum_xz; Mtx[7] = sum_yz; Mtx[8] = sum_zz;
	//Mtx[0] = sum_xx; Mtx[1] = sum_xy; Mtx[2] = sum_x;
	//Mtx[3] = sum_xy; Mtx[4] = sum_yy; Mtx[5] = sum_y;
	//Mtx[6] = sum_x; Mtx[7] = sum_y; Mtx[8] = FitPointNum;
	MatrixN3Inv(Mtx,InvMtx);
	Plane[0] = sum_x*InvMtx[0]+sum_y*InvMtx[1]+sum_z*InvMtx[2];
	Plane[1] = sum_x*InvMtx[3]+sum_y*InvMtx[4]+sum_z*InvMtx[5];
	Plane[2] = sum_x*InvMtx[6]+sum_y*InvMtx[7]+sum_z*InvMtx[8];
	//Plane[0] = sum_xz*InvMtx[0]+sum_yz*InvMtx[1]+sum_z*InvMtx[2];
	//Plane[1] = sum_xz*InvMtx[3]+sum_yz*InvMtx[4]+sum_z*InvMtx[5];
	//Plane[2] = sum_xz*InvMtx[6]+sum_yz*InvMtx[7]+sum_z*InvMtx[8];
	VectorUnitization(Plane);
	Plane[3] = sum_x/FitPointNum;
	Plane[4] = sum_y/FitPointNum;
	Plane[5] = sum_z/FitPointNum;
	return true;
}

float CheckPoleStyle(int CheckPN,float* CheckPointP)
{
	int i=0;
	int k=0;
	float PlaneCurvature = 0;  //��������ֵ���˼��ֲ��������ƽ���Բ����֮������ʣ�
	float OutlineP[10000][3]={0};
	float Plane[2][3]={0};
	//ȡ���������ֵ
	for (i=0;i<CheckPN;i++)
	{
		for (k=0;k<3;k++)
		{
			OutlineP[i][k] = *(CheckPointP+i*3+k);
		}
	}
	//ȷ������ƽ��ķ�����������һ��
	PlaneFitting(CheckPN,CheckPointP,Plane[0]);

	//�����㵽ƽ������ƽ��ֵ
	for (i=0;i<CheckPN;i++)
	{	
		//����ƽ���ȽϺã���Ϊ���ڷ���ƽ����˵
		//һ���Ϊ0.����ƽ�����С������Բ����
		//�����Ϊ1.����ƽ���������������������
		PlaneCurvature += pow(PtoPlaneDis(true,Plane[0],Plane[1],OutlineP[i]),2);
		//PtoPlaneDis(true,Plane[0],Plane[1],OutlineP[i]);
	}
	PlaneCurvature = abs(PlaneCurvature)/CheckPN;
	return PlaneCurvature;
}

void CheckPtoLine(int CheckPN, float* CheckP, float* LineDir, float* LineP, float* R)
{
	int i=0;
	int k=0;
	float OutlineP[10000][3]={0};
	//ȡ���������ֵ
	R[0]=2000;
	R[1]=0;
	int num=0;
	for (i=0;i<CheckPN;i++)
	{
		for (k=0;k<3;k++)
		{
			OutlineP[num][k] = *(CheckP+i*3+k);
		}
		if (OutlineP[num][2] == 0)  //�ų��˼����������к���Ŀհ�
		{
			num--;
		}
		num++;
	}
	float TempD;
	for (i=0;i<num-1;i++)
	{
		TempD = PtoLineDis(OutlineP[i],LineDir,LineP);
		if (R[0]>TempD)
		{
			R[0] = TempD;
		}
		if (R[1]<TempD)
		{
			R[1] = TempD;
		}
	}

}//

int main(int argc, char* argv[])
{
	//��ʼ��
	int i=0,j=0,k=0,q=0;   //ѭ������
	int PoleN = 0;
	int g_PolesNum = 0;
	static float Frame3D[480][640][3];  //���������ͼ��ת����3D��ʵ�ռ���ơ�
	float CliffP[30][5];//���г��ֵ����ɸ����µ㣨�������о�����ĵ㣩�Լ��õ������ͼ���еĶ�ά����//30�����µ��Ƿ��㹻��������
	int CliffPMark[30];//���µ�������־,1��ʾ����ǰһ��Զ����0��ʾ������һ��Զ��
	int CliffPn = 0;  //ÿ�����µ����
	float PolesStartBoundaryP[5][5]; //��⵽�˼���Ŀ���ʱ�ĸ˼���һ���߽�㣨�����µ㣩�Լ��õ������ͼ���еĶ�ά����
	float PolesEndBoundaryP[5][5]; //��⵽�˼���Ŀ���ʱ�ĸ˼��ڶ����߽�㣨�����µ㣩�Լ��õ������ͼ���еĶ�ά����
	float PolesStartBoundaryPTemp[5][5];
	float PolesEndBoundaryPTemp[5][5];
	int PoleNTemp = 0;
	int InPlane;
	bool DeteSymbol;
	int PoleLenghtTemp[DETECT_POLE_NUM_MAX]; 
	float EdgeToBackLineDis[DetPoleNum][2]={{1000,1000},{1000,1000},{1000,1000},{1000,1000},{1000,1000}};
	float EBLDMax[10];  //������߽�㵽���м�����С�������Ǳ߸���������С����������һ����
	float PCentralLineTemp[DETECT_POLE_NUM_MAX][2][3];//GU  �ڶ���0��ʾ����1��ʾ���ĵ�

    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read("/home/yake/Temp/simple_scene_only_cylinder.pcd", *cloud);

    for(int i=0;i<cloud->height;++i)
    {
        for(int j=0;j<cloud->width;++j)
        {
            Frame3D[i][j][0]=cloud->at(j,i).x*1000.0;
            Frame3D[i][j][1]=cloud->at(j,i).y*1000.0;
            Frame3D[i][j][2]=cloud->at(j,i).z*1000.0;
        }
    }

    for(int i=1;i<cloud->height;++i)
    {
        for(int j=1;j<cloud->width;++j)
        {
            float z = Frame3D[i][j][2];
            if(pcl_isnan(z))
            {
                Frame3D[i][j][0]=Frame3D[i][j-1][0];
                Frame3D[i][j][1]=Frame3D[i][j-1][1];
                Frame3D[i][j][2]=Frame3D[i][j-1][2];
            }
        }
    }


    //View test
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_view(new pcl::PointCloud<pcl::PointXYZ>);
//    cloud_view->width=640;
//    cloud_view->height=480;
//    cloud_view->points.resize(640*480);
//    for(int i=0;i<cloud_view->height;++i)
//    {
//        for(int j=0;j<cloud_view->width;++j)
//        {
//            cloud_view->at(j,i).x=Frame3D[i][j][0];
//            cloud_view->at(j,i).y=Frame3D[i][j][1];
//            cloud_view->at(j,i).z=Frame3D[i][j][2];
//        }
//    }
//    pcl::visualization::PCLVisualizer v("view");
//    v.addPointCloud(cloud_view);
//    v.spin();

    double t = cv::getTickCount();
	for (i=5;i<35;i++)  //��10��5����40��35�����У���ȡ�˼���Ŀ�����У��˼��ĸ�����PoleN���ͱ߽��PolesStartBoundaryP[i][k]��PolesEndBoundaryP[i][k]
	{
		//�������µ���
		for (j=0;j<640;j++)
		{
//            if(i=30)
//            {
//                cout<<Frame3D[i][j][2]<<endl;
//            }
			if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])>100)
			{
				for (k=0;k<3;k++)
				{
					CliffP[CliffPn][k] = Frame3D[i][j+1][k];  //��¼�����нϽ��ĵ�
					CliffPMark[CliffPn] = 1;
				}
				CliffP[CliffPn][3] = i;
				CliffP[CliffPn][4] = j+1;
				CliffPn++;
			}
			else if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])<-100)
			{
				for (k=0;k<3;k++)
				{
					CliffP[CliffPn][k] = Frame3D[i][j][k]; //��¼�����нϽ��ĵ�
					CliffPMark[CliffPn] = 0;
				}
				CliffP[CliffPn][3] = i;
				CliffP[CliffPn][4] = j;
				CliffPn++;
			}
		}
		///////////////////////////////////
		//ÿ�����һ�����µ�󣬶Լ�⵽�����µ���������
		for (j=0;j<CliffPn-1;j++)
		{
			if (CliffPMark[j]==1 && CliffPMark[j+1]==0)//���µ�������־,1��ʾ�����õ������ǰһ��Ͻ�����0��ʾ�����õ�����ں�һ��Ͻ���
				//��������ʾΪĳһ����������������µ�
			{
				//����Ƿ�����ɼг�����
				if (P2Distance(CliffP[j],CliffP[j+1])<80*1.414 && P2Distance(CliffP[j],CliffP[j+1])>45)
				{
					for (k=0;k<5;k++)
					{
						PolesStartBoundaryPTemp[PoleNTemp][k] = CliffP[j][k];//�Լ�⵽�����µ��з��ϼгָ˼�Ҫ�����������
						PolesEndBoundaryPTemp[PoleNTemp][k] = CliffP[j+1][k];
					}
					PoleNTemp++;
				}
			}
		}
		CliffPn = 0;//���µ���Ŀ����
		//////////////////////////////////////
		//��ȡ�˼������ʱ�ĸ˼������ͱ�����ϣ���¼��ά����Ͷ�ά�������꣩
		if (PoleNTemp>PoleN)
		{
			PoleN = PoleNTemp;
			for (i=0;i<PoleN;i++)
			{
				for (k=0;k<5;k++)
				{
					PolesStartBoundaryP[i][k] = PolesStartBoundaryPTemp[i][k];
					PolesEndBoundaryP[i][k] = PolesEndBoundaryPTemp[i][k];
				}
			}
		}
		PoleNTemp = 0;//��ʱ�˼���Ŀ����
	}

	if (PoleN > 5)//HU  Ϊʲô���ֻ��5���ˣ�����

	{
        //printf("More than max num\n");
		return -1;
	}

	//�˼���Ϣ�ṹ��
	PoleInfo poleINFO[5];
	(void)memset(poleINFO, 0, sizeof(poleINFO));

	//////////////////////////////////////////////////////////////////////////
	//���ÿһ���˼���������
	static float OutlineP[DetPoleNum][480][200][3] = {0};  //���ͼ����ͬһ�У�DepthPixelY��ȵģ�ɨ��˼�������
	//���ͼ����ĳ���˵�ĳ���Ա�Ե��ĳ�е����ά����
	float P_BoundaryP[DetPoleNum][480][2][5] = {0};  //�˼������߽��
	int PoleJudge[40][2];//�洢��˼��߽�㣨���µ㣩��ά���������е�j�Լ����µ������
	int PoleJudgeN =0;//��˼��߽�㣨���µ㣩����
	int PoleRowN[DetPoleNum] = {0};//����ĳ���˵ĵ�������

	for (int Pole_i=0;Pole_i<PoleN;Pole_i++)
	{
		for (k=0;k<5;k++)//�������ϸ˼���Ŀ����е����µ���Ϣ
		{
			P_BoundaryP[Pole_i][0][0][k] = PolesStartBoundaryP[Pole_i][k];
			P_BoundaryP[Pole_i][0][1][k] = PolesEndBoundaryP[Pole_i][k];
		}
		for (i=PolesStartBoundaryP[Pole_i][3];i<480;i++)  //�ӵ�Pole_i���˼��ĵ�PolesStartBoundaryP[Pole_i][3]�����ؿ�ʼɨ��
		{
			//��ÿһ�е�̽Ѱ��Χ������0~640��
			if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4]-50<0)
			{
				P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4] = 50;
			}
			if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4]+50>640)
			{
				P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4] = 590;
			}
			////////////////////////////////////////////////////////
			//����ÿ���з��ϼг�Ҫ��ĸ˼��������������µ�Ϊ��ʼ����
			//���ø˼��ڸ����е����µ�
			for (j=P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4]-50;j<P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4]+50;j++)
			{
				if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])>CliffDis)
				{
					PoleJudge[PoleJudgeN][0] = j+1;//��һ���е���˼��߽�㣨��߽磩�Ķ�ά���������е�j
					PoleJudge[PoleJudgeN][1] = 1;//1��ʾ�����õ������ǰһ��Ͻ�����0��ʾ�����õ�����ں�һ��Ͻ���
					PoleJudgeN++;
				}
				else if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])<-CliffDis)
				{
					PoleJudge[PoleJudgeN][0] = j;//��һ���е���˼��߽�㣨�ұ߽磩�Ķ�ά���������е�j
					PoleJudge[PoleJudgeN][1] = 0;//1��ʾ�����õ������ǰһ��Ͻ�����0��ʾ�����õ�����ں�һ��Ͻ���
					PoleJudgeN++;
				}
			}
			//////////////////////////////////////////////////////
			if (PoleJudgeN>1)
			{
				int mar=0;//�˼���Ŀ
				for (j=0;j<PoleJudgeN-1;j++)
				{
					//�Լ�⵽����˼��߽���з��ϼгָ˼�Ҫ����������ϣ���ͬһ���˼�������������߽�����һ��������ͳһ��ʾ����
					//�洢���ͼ����ͬһ��ɨ��˼�������
					if (PoleJudge[j][1]==1 && PoleJudge[j+1][1]==0
						&&P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])<80*1.414 
						&& P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])>45)
					{
						mar++;
						//�洢������˼��߽�㣨���µ㣩����ά����Ͷ�ά��������
						for (k=0;k<3;k++)
						{
							P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][k] = Frame3D[i][PoleJudge[j][0]][k];
							P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][k] = Frame3D[i][PoleJudge[j+1][0]][k];
						}
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][3] = i;
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][3] = i;
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][0][4] = PoleJudge[j][0];
						P_BoundaryP[Pole_i][PoleRowN[Pole_i]+1][1][4] = PoleJudge[j+1][0];
						///////////////////////////////////////////////////////////////
						int tempp=0;//���ڸø˼��ĸ�������������
						float DisMax = Frame3D[i][PoleJudge[j][0]][2];
						if (Frame3D[i][PoleJudge[j+1][0]][2]>Frame3D[i][PoleJudge[j][0]][2])//ȷ��������˼��߽�������ֵ�ϴ���
						{
							DisMax = Frame3D[i][PoleJudge[j+1][0]][2];
						}
						for (q=PoleJudge[j][0];q<PoleJudge[j+1][0];q++)//�洢���ͼ����ͬһ��ɨ��˼�������
						{
							if (Frame3D[i][q][2]<DisMax && Frame3D[i][q][2]>(DisMax-80*1.414))
							{
								for (k=0;k<3;k++)
								{
									OutlineP[Pole_i][PoleRowN[Pole_i]][tempp][k] = Frame3D[i][q][k]; 	
								}
								tempp++;
							}
						}
						if (tempp<((PoleJudge[j+1][0]-PoleJudge[j][0])*0.8))//
						{
							PoleRowN[Pole_i]--;
						}
					}
				}
				if (mar==0)
				{
					PoleRowN[Pole_i]--;
				}
			} 
			else
			{
				PoleRowN[Pole_i]--;
			}
			PoleJudgeN = 0;
			PoleRowN[Pole_i]++;
		}
	}
	////////////////////////////////////////////////////////////////////


	//Ѱ�Ҹ˼�������ߣ�����
	////////////////////////////////////////////////////////////////////
	float NearestLineP[DetPoleNum][480][3]={0}; //ÿһ��������ĵ㣨���㣩
	int NearestLinePN[5][480];   //ÿһ��������ĵ��ڸ����е�λ��

		for (i=0;i<PoleN;i++)
	{
		for (j=0;j<PoleRowN[i];j++)
		{
			NearestLineP[i][j][2] = 1600;//�������Щ���Ա�֤ȡ��ÿ���е������
			for(int rowPN=0;rowPN<300;rowPN++)
			{
				if (OutlineP[i][j][rowPN][2]==0)
				{
					break;
				}
				//rowPN��ʾ���ڸ˼��ĵ��Ƹ�����������Ҫ�������ڸ˵����б���㣬�ҳ������
				if (NearestLineP[i][j][2]>OutlineP[i][j][rowPN][2])
				{
					NearestLineP[i][j][0]=OutlineP[i][j][rowPN][0];
					NearestLineP[i][j][1]=OutlineP[i][j][rowPN][1];
					NearestLineP[i][j][2]=OutlineP[i][j][rowPN][2];
					NearestLinePN[i][j] = rowPN;
				}
			}
		}
	}
	///////////////////////////////////////////////////////////

	//�жϼ����Ƿ�����ƽ����
	///////////////////////////////////////////////////////////
	for (i=0;i<PoleN;i++)
	{
		j=0;
		//�����м������еĵ㵽����ľ���С��15�ĵ������
		while(P2Distance(OutlineP[i][PoleRowN[i]/2][j],NearestLineP[i][PoleRowN[i]/2])<15)
		{
			j++;
		}
		int k=j;
		if (k>198)
		{
			k=198;
		}
		//�����м������еĵ㵽����ľ������15�ĵ������
		while(P2Distance(OutlineP[i][PoleRowN[i]/2][j],NearestLineP[i][PoleRowN[i]/2])>15)
		{
			j++;
		}
		if (j>198)
		{
			j=198;
		}
		if ((abs(NearestLineP[i][PoleRowN[i]/2][2]-OutlineP[i][PoleRowN[i]/2][j][2])<1.5)
			||(abs(NearestLineP[i][PoleRowN[i]/2][2]-OutlineP[i][PoleRowN[i]/2][k][2])<1.5))
		{
			InPlane++;
			if (InPlane>20)
			{
				//AfxMessageBox("ƽ����");
				InPlane = 0;
				DeteSymbol = false;
			}
			return -1;

		}
	}

	//�����������ֱ��
	///////////////////////////////////////////////////////////////
	float NearestLine[DetPoleNum][2][3];
	for (i=0;i<PoleN;i++)
	{
		BoundaryLineFit(PoleRowN[i],NearestLineP[i][0],NearestLine[i][0],NearestLine[i][1]);
	}

	//��ֱ�ȣ����������
	for (i=0;i<PoleN;i++)
	{
		poleINFO[i].BendDeg = 0;//��������������P24
		for (j=0;j<PoleRowN[i];j++)
		{
			poleINFO[i].BendDeg += PtoLineDis(NearestLineP[i][j],NearestLine[i][0],NearestLine[i][1]);// �㵽ֱ�ߵľ��뺯��
		}
		poleINFO[i].BendDeg = poleINFO[i].BendDeg/PoleRowN[i];
	}

	//�����˼�����
	for (i=0;i<PoleN;i++)
	{
		poleINFO[i].PoleLenght = P2Distance(NearestLineP[i][0],NearestLineP[i][PoleRowN[i]]);
		PoleLenghtTemp[i]=poleINFO[i].PoleLenght;
	}
	//////////////////////////////////////////////////////////////////////////

	//�ҳ���Ե���뱳��������ƽ��f(x,y)=0����ƽ�棩����ĵ㣬�Ƚ����߱�Ե���������С
	//////////////////////////////////////////////////////////////////////////

	//��ȡ�а�֮����ж�����
	///////////////////////////////////////////////////////////////////////////
	int checkhigh[5]={0};//�˳���70mm֮�ڵĵ�������
	for (i=0;i<PoleN;i++)
	{
		//�˳���70֮�ڵĵ�������
		while (P2Distance(OutlineP[i][(PoleRowN[i]-checkhigh[i])/2][10],OutlineP[i][(PoleRowN[i]+checkhigh[i])/2][10])<70)//
		{
			checkhigh[i] = checkhigh[i]+2;
		}
	}

	float tempdis;
	for (i=0;i<PoleN;i++)
	{
		//��70mm�˳��ķ�Χ����Ѱ��߽�㵽���м�����������
		for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)//j=0;j<Xcounts;j++)
		{
			//������ʼ��߽�㵽���еļ���ľ���
			tempdis = P2Distance(P_BoundaryP[i][j][0],NearestLineP[i][j]);
			//��¼�������
			if (EdgeToBackLineDis[i][0]>tempdis)
			{
				EdgeToBackLineDis[i][0] = tempdis;
			}
			//������ֹ��߽�㵽���еļ���ľ���
			tempdis = P2Distance(P_BoundaryP[i][j][1],NearestLineP[i][j]);
			//��¼�������
			if (EdgeToBackLineDis[i][1]>tempdis)
			{
				EdgeToBackLineDis[i][1] = tempdis;
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////
	//ʹ�ü�ƽ�淽���������������һ��
	/////////////////////////////////////////////////////////////////////////
	int CheckStartFlag[5]; //��Ϊ0��ʾ��ߴ�Ϊ1��ʾ�ұߴ�
	for (i=0;i<PoleN;i++)
	{
		EBLDMax[i] = EdgeToBackLineDis[i][0];
		if (EdgeToBackLineDis[i][0]>EdgeToBackLineDis[i][1])
		{
			CheckStartFlag[i] = 0;
		}
		else
		{
			CheckStartFlag[i] = 1;
			EBLDMax[i] = EdgeToBackLineDis[i][1];
		}
	}

	int CheckAreaP[DetPoleNum][200][3] = {0};  
	for (i=0;i<PoleN;i++)
	{
		int k=0;
		if (CheckStartFlag[i]==0)  //��ߴ�
		{
			for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)
			{
				CheckAreaP[i][k][0] = 0;//�˼�ĳһ�У�k���������ڼ����������Ҳ������С���루EBLDMax[i]-2���ĵ�ĸ���
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][0]],NearestLineP[i][j])>(EBLDMax[i]-2))
				{
					CheckAreaP[i][k][0] ++;
				}
				CheckAreaP[i][k][1] = 0;//�˼�ĳһ�У�k���������ڼ����������Ҳ���ھ��루11���ĵ�ĸ���
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][1]],NearestLineP[i][j])>11)
				{
					CheckAreaP[i][k][1] ++;
				}
				CheckAreaP[i][k][2] = j;//�˼�ĳһ�е���������
				k++;
			}
		}
		else if (CheckStartFlag[i]==1) //�ұߴ�
		{
			for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)
			{
				CheckAreaP[i][k][0] = NearestLinePN[i][j];
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][0]],NearestLineP[i][j])<(EBLDMax[i]-2))
				{
					CheckAreaP[i][k][0] ++;
				}
				CheckAreaP[i][k][1] = NearestLinePN[i][j];
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][1]],NearestLineP[i][j])<11)
				{
					CheckAreaP[i][k][1] ++;
				}
				CheckAreaP[i][k][2] = j;
				k++;
			}
		}
	}

	//�жϷ���Բ��,
	////////////////////////////////////////////////////////////////////////////////
	float CheckPoint[DetPoleNum][5000][3];//ĳ���˼�ĳ���о�����м����ڣ�(EBLDMax[i]-2)��11����Χ�ڵĵ������
	int CheckTriangleN[DetPoleNum][3];   //������������������λ��
	for (i=0;i<PoleN;i++)
	{
		CheckTriangleN[i][0] = 0;
		int checkn=0;
		for (int k=0;k<checkhigh[i];k++)
		{		
			if (CheckAreaP[i][k][0]>CheckAreaP[i][k][1])
			{
				for (j=CheckAreaP[i][k][1];j<CheckAreaP[i][k][0];j++)
				{
					for (int z=0;z<3;z++)
					{
						CheckPoint[i][checkn][z] = OutlineP[i][CheckAreaP[i][k][2]][j][z];
					}
					checkn++;
				}
				if (k==0)
				{
					CheckTriangleN[i][1] = checkn-1;
				}
			}
			else
			{
				for (j=CheckAreaP[i][k][0];j<CheckAreaP[i][k][1];j++)
				{
					for (int z=0;z<3;z++)
					{
						CheckPoint[i][checkn][z] = OutlineP[i][CheckAreaP[i][k][2]][j][z];
					}
					checkn++;
				}
				if (k==0)
				{
					CheckTriangleN[i][1] = checkn-1;
				}
			}
		}
		CheckTriangleN[i][2] = checkn-1;
	}

	float CheckPlane[DetPoleNum][2][3]={0};  //����ƽ�淽��
	float CheckLine[DetPoleNum][3]={0};
	int RSJ[DETECT_POLE_NUM_MAX];//HU
	for (i=0;i<PoleN;i++)
	{
		if (CheckPoleStyle(CheckTriangleN[i][2],CheckPoint[i][0])>0.26)
		{
			poleINFO[i].PoleStyle = 1;  //Բ��
			RSJ[i] = 1;
		}
		else
		{
			//poleINFO[i].PoleStyle = -1;  //����
			//RSJ[i] = -1;
			//PlaneFitting(CheckTriangleN[i][2],CheckPoint[i][0],CheckPlane[i][0]);
			//CheckLine[i][0] = CheckPlane[i][0][1]*NearestLine[i][0][2] - CheckPlane[i][0][2]*NearestLine[i][0][1];
			//CheckLine[i][1] = CheckPlane[i][0][2]*NearestLine[i][0][0] - CheckPlane[i][0][0]*NearestLine[i][0][2];
			//CheckLine[i][2] = CheckPlane[i][0][0]*NearestLine[i][0][1] - CheckPlane[i][0][1]*NearestLine[i][0][0];
			//VectorUnitization(CheckLine[i]);
		}
	}

	//����˼������߷��̺͸˼��ߴ�
	for (i=0;i<PoleN;i++)
	{
		int openglk;
		float PolesCentre[3];
		float PolesTempCentre[3];	
		float tempD;
		float R[2];
		//Բ��
		if (poleINFO[i].PoleStyle==1)
		{
			///////////////////////////
			poleINFO[i].Cylinder_Dis = 1000;
			for (k=0;k<3;k++)
			{
				poleINFO[i].PoleFunc[0][k] = NearestLine[i][0][k];
				poleINFO[i].Pose[k] = 0;

				PCentralLineTemp[i][0][k] = NearestLine[i][0][k];//GU
			}
			///////////////////////////////Ѱ�Ҹ˼����ߵ����Ż�����GU
			for (j=0;j<11;j++)
			{
				PolesTempCentre[0] = NearestLine[i][1][0]+(10-2*j);//��Χ����10
				PolesTempCentre[1] = NearestLine[i][1][1];
				PolesTempCentre[2] = NearestLine[i][1][2]+10;//
				for (k=0;k<50/StepLen;k++)
				{
					PolesTempCentre[2] = PolesTempCentre[2]+StepLen;
					CheckPtoLine(CheckRowN*200,OutlineP[i][(PoleRowN[i]-CheckRowN)/2][0],NearestLine[i][0],PolesTempCentre,R);
					//////////////////////////////////
					//////////////////////////////////////////////////////////////////////////////////////
					tempD = R[1]-R[0];
					if (poleINFO[i].Cylinder_Dis>tempD)
					{
						poleINFO[i].Cylinder_Dis = tempD;
						for (k=0;k<3;k++)
						{
							PolesCentre[k] = PolesTempCentre[k];//�ô���ø˼����е�λ��P25
						}
					}
				}

			}
			for (k=0;k<3;k++)
			{
				poleINFO[i].PoleFunc[1][k] = PolesCentre[k];

				PCentralLineTemp[i][1][k] = PolesCentre[k];//GU
			}
			//�˼��뾶
			poleINFO[i].PoleDiameter = PtoLineDis(PolesCentre,NearestLine[i][0],NearestLine[i][1]);
			////PolesDim[i] = PolesCentre[2]-NearestLine[i][1][2];
			//PCentralLine[i][0][0] = PolesCentre[0]+220*NearestLine[i][0][0];
			//PCentralLine[i][0][1] = PolesCentre[1]+220*NearestLine[i][0][1];
			//PCentralLine[i][0][2] = PolesCentre[2]+220*NearestLine[i][0][2];

			////for (int k = 0; k < 3; k++)//GU
			////{
			////	PCentralLineTemp[i][0][k] = PCentralLine[i][0][k];
			////}

			//PointCoordinateC(&PUntifyMtx,PCentralLine[i][0],PCentralLine[i][0]);
			//for (openglk=0;openglk<3;openglk++)
			//{
			//	PCentralLine[i][0][openglk] = PCentralLine[i][0][openglk]/1000.0;
			//}
			//PCentralLine[i][1][0] = PolesCentre[0]-220*NearestLine[i][0][0];
			//PCentralLine[i][1][1] = PolesCentre[1]-220*NearestLine[i][0][1];
			//PCentralLine[i][1][2] = PolesCentre[2]-220*NearestLine[i][0][2];

			////for (int k = 0; k < 3; k++)//GU
			////{
			////	PCentralLineTemp[i][1][k] = PCentralLine[i][1][k];
			////}

			//PointCoordinateC(&PUntifyMtx,PCentralLine[i][1],PCentralLine[i][1]);
			//for (openglk=0;openglk<3;openglk++)
			//{
			//	PCentralLine[i][1][openglk] = PCentralLine[i][1][openglk]/1000.0;
			//	PonCentline[i][openglk] = PolesCentre[openglk]/1000.0;
			//}
		}
		
		if (poleINFO[i].Cylinder_Dis>5 && poleINFO[i].PoleStyle==Square)
		{
			poleINFO[i].PoleStyle==Square;
		}
		else
			poleINFO[i].PoleStyle==Round;
	}

	g_PolesNum = PoleN;
	//���㾲̬����
	memset(Frame3D,0,480*640*3*sizeof(float));
	memset(OutlineP,0,DetPoleNum*480*200*3*sizeof(float));
    t=(cv::getTickCount()-t)/cv::getTickFrequency();
    cout<<"Time: "<<t<<endl;

	return 0;
}

