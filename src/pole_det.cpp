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
	char PoleStyle;   //杆件类型：1表示圆杆，-1表示方杆
	float PoleFunc[2][3];  //杆件轴线方程
	float PoleDiameter;  //杆件直径
	unsigned int PoleLenght; //杆长
	float Pose[3];   //杆件周向姿态
	float BendDeg;   //杆件弯曲度
	float Cylinder_Dis; //杆件定义中两圆柱面之间的间隙值
}PoleInfo;

float P2Distance(const float* P1,const float* P2)
{
	float p2distance;
	p2distance = sqrt((P1[0]-P2[0])*(P1[0]-P2[0])+(P1[1]-P2[1])*(P1[1]-P2[1])+(P1[2]-P2[2])*(P1[2]-P2[2]));
	return p2distance;
}

/*void BoundaryLineFit(int LinePN, float* LineP, float* LineDir, float* OnLineP)
功能：“脊”线拟合算法
输入：LinePN 像素点行数；LineP “脊”点坐标
输出：LineDir “脊”线的方向向量；OnLineP “脊”线段的中点
返回：
*/
void BoundaryLineFit(int LinePN, float* LineP, float* LineDir, float* OnLineP)//“脊”线拟合算法，HU论文P24
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
	//二次项
	double sum_xy = 0;
	double sum_yy = 0;
	double sum_zy = 0;
	//一次项
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
	//求直线的方向向量和一点
	//方向向量LineDir
	LineDir[0] = (LinePN*sum_xy-sum_x*sum_y)/(LinePN*sum_yy-sum_y*sum_y);
	LineDir[1] = 1;
	LineDir[2] = (LinePN*sum_zy-sum_z*sum_y)/(LinePN*sum_yy-sum_y*sum_y);
	//直线上一点OnLineP
	// 	OnLineP[0] = (sum_x-LineDir[0]*sum_y)/LinePN;
	// 	OnLineP[1] = 0;
	// 	OnLineP[2] = (sum_z-LineDir[2]*sum_y)/LinePN;
	OnLineP[0] = sum_x/LinePN;
	OnLineP[1] = sum_y/LinePN;
	OnLineP[2] = sum_z/LinePN;
	//剔除错误点
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
	//剔除后的直线方程
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
		//一次项
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
		//求直线的方向向量和一点
		//方向向量LineDir
		LineDir[0] = (LinePN*sum_xy-sum_x*sum_y)/(LinePN*sum_yy-sum_y*sum_y);//与论文中有差异
		LineDir[1] = 1;
		LineDir[2] = (LinePN*sum_zy-sum_z*sum_y)/(LinePN*sum_yy-sum_y*sum_y);
		//直线上一点OnLineP
		// 		OnLineP[0] = (sum_x-LineDir[0]*sum_y)/LinePN;
		// 		OnLineP[1] = 0;
		// 		OnLineP[2] = (sum_z-LineDir[2]*sum_y)/LinePN;
		OnLineP[0] = sum_x/LinePN;
		OnLineP[1] = sum_y/LinePN;
		OnLineP[2] = sum_z/LinePN;
	}
}

// 点到直线的距离函数
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

// 点到平面的距离 , 带有正负号
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

// 三阶矩阵求逆
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

// 向量单位化
void VectorUnitization(float* Vector)
{
	float Vectorlength = sqrt(Vector[0]*Vector[0]+Vector[1]*Vector[1]+Vector[2]*Vector[2]);
	Vector[0] = Vector[0]/Vectorlength;
	Vector[1] = Vector[1]/Vectorlength;
	Vector[2] = Vector[2]/Vectorlength;
}

//Plane 里面的四个量分别是（A,B,C）,Ax+By+Cz+1=0
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
	//二次项
	double sum_xx = 0;
	double sum_yy = 0;
	double sum_zz = 0;
	double sum_xy = 0;
	double sum_xz = 0;
	double sum_yz = 0;
	//一次项
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
	float PlaneCurvature = 0;  //函数返回值，杆件局部面相对于平面和圆弧面之间的曲率，
	float OutlineP[10000][3]={0};
	float Plane[2][3]={0};
	//取出轮廓点的值
	for (i=0;i<CheckPN;i++)
	{
		for (k=0;k<3;k++)
		{
			OutlineP[i][k] = *(CheckPointP+i*3+k);
		}
	}
	//确定算子平面的法向量与其上一点
	PlaneFitting(CheckPN,CheckPointP,Plane[0]);

	//各个点到平面距离的平均值
	for (i=0;i<CheckPN;i++)
	{	
		//加入平方比较好，因为对于方杆平面来说
		//一般多为0.几，平反后更小，对于圆杆面
		//则距离为1.几，平方变大，这样二者区分明显
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
	//取出轮廓点的值
	R[0]=2000;
	R[1]=0;
	int num=0;
	for (i=0;i<CheckPN;i++)
	{
		for (k=0;k<3;k++)
		{
			OutlineP[num][k] = *(CheckP+i*3+k);
		}
		if (OutlineP[num][2] == 0)  //排除杆件轮廓数组中后面的空白
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
	//初始化
	int i=0,j=0,k=0,q=0;   //循环变量
	int PoleN = 0;
	int g_PolesNum = 0;
	static float Frame3D[480][640][3];  //将整个深度图像转化成3D真实空间点云。
	float CliffP[30][5];//单行出现的若干个悬崖点（两个点中距离近的点）以及该点在深度图像中的二维坐标//30个悬崖点是否足够？？？？
	int CliffPMark[30];//悬崖点正负标志,1表示正（前一点远），0表示负（后一点远）
	int CliffPn = 0;  //每行悬崖点个数
	float PolesStartBoundaryP[5][5]; //检测到杆件数目最多时的杆件第一个边界点（正悬崖点）以及该点在深度图像中的二维坐标
	float PolesEndBoundaryP[5][5]; //检测到杆件数目最多时的杆件第二个边界点（正悬崖点）以及该点在深度图像中的二维坐标
	float PolesStartBoundaryPTemp[5][5];
	float PolesEndBoundaryPTemp[5][5];
	int PoleNTemp = 0;
	int InPlane;
	bool DeteSymbol;
	int PoleLenghtTemp[DETECT_POLE_NUM_MAX]; 
	float EdgeToBackLineDis[DetPoleNum][2]={{1000,1000},{1000,1000},{1000,1000},{1000,1000},{1000,1000}};
	float EBLDMax[10];  //两侧拟边界点到该行脊点最小距离中那边更大（两边最小距离中最大的一个）
	float PCentralLineTemp[DETECT_POLE_NUM_MAX][2][3];//GU  第二项0表示方向，1表示中心点

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
	for (i=5;i<35;i++)  //第10（5）到40（35）行中，求取杆件数目最多的行，杆件的个数（PoleN）和边界点PolesStartBoundaryP[i][k]、PolesEndBoundaryP[i][k]
	{
		//单行悬崖点检测
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
					CliffP[CliffPn][k] = Frame3D[i][j+1][k];  //记录两点中较近的点
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
					CliffP[CliffPn][k] = Frame3D[i][j][k]; //记录两点中较近的点
					CliffPMark[CliffPn] = 0;
				}
				CliffP[CliffPn][3] = i;
				CliffP[CliffPn][4] = j;
				CliffPn++;
			}
		}
		///////////////////////////////////
		//每检查完一行悬崖点后，对检测到的悬崖点重新整合
		for (j=0;j<CliffPn-1;j++)
		{
			if (CliffPMark[j]==1 && CliffPMark[j+1]==0)//悬崖点正负标志,1表示正（该点相对于前一点较近），0表示负（该点相对于后一点较近）
				//该条件表示为某一物体的左右两个悬崖点
			{
				//检查是否满足可夹持条件
				if (P2Distance(CliffP[j],CliffP[j+1])<80*1.414 && P2Distance(CliffP[j],CliffP[j+1])>45)
				{
					for (k=0;k<5;k++)
					{
						PolesStartBoundaryPTemp[PoleNTemp][k] = CliffP[j][k];//对检测到的悬崖点中符合夹持杆件要求的重新整合
						PolesEndBoundaryPTemp[PoleNTemp][k] = CliffP[j+1][k];
					}
					PoleNTemp++;
				}
			}
		}
		CliffPn = 0;//悬崖点数目清零
		//////////////////////////////////////
		//求取杆件数最多时的杆件个数和标记整合（记录三维坐标和二维像素坐标）
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
		PoleNTemp = 0;//临时杆件数目清零
	}

	if (PoleN > 5)//HU  为什么最多只有5根杆？？？

	{
        //printf("More than max num\n");
		return -1;
	}

	//杆件信息结构体
	PoleInfo poleINFO[5];
	(void)memset(poleINFO, 0, sizeof(poleINFO));

	//////////////////////////////////////////////////////////////////////////
	//求出每一根杆件的外轮廓
	static float OutlineP[DetPoleNum][480][200][3] = {0};  //深度图像中同一行（DepthPixelY相等的）扫描杆件轮廓点
	//深度图像中某根杆的某行自边缘起某列点的三维坐标
	float P_BoundaryP[DetPoleNum][480][2][5] = {0};  //杆件轮廓边界点
	int PoleJudge[40][2];//存储拟杆件边界点（悬崖点）二维像素坐标中的j以及悬崖点的正负
	int PoleJudgeN =0;//拟杆件边界点（悬崖点）个数
	int PoleRowN[DetPoleNum] = {0};//属于某根杆的点云行数

	for (int Pole_i=0;Pole_i<PoleN;Pole_i++)
	{
		for (k=0;k<5;k++)//重新整合杆件数目最多行的悬崖点信息
		{
			P_BoundaryP[Pole_i][0][0][k] = PolesStartBoundaryP[Pole_i][k];
			P_BoundaryP[Pole_i][0][1][k] = PolesEndBoundaryP[Pole_i][k];
		}
		for (i=PolesStartBoundaryP[Pole_i][3];i<480;i++)  //从第Pole_i根杆件的第PolesStartBoundaryP[Pole_i][3]行像素开始扫描
		{
			//将每一行的探寻范围控制在0~640内
			if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4]-50<0)
			{
				P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4] = 50;
			}
			if (P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4]+50>640)
			{
				P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4] = 590;
			}
			////////////////////////////////////////////////////////
			//（以每行中符合夹持要求的杆件的左右两个悬崖点为起始，）
			//检测该杆件在该行中的悬崖点
			for (j=P_BoundaryP[Pole_i][PoleRowN[Pole_i]][0][4]-50;j<P_BoundaryP[Pole_i][PoleRowN[Pole_i]][1][4]+50;j++)
			{
				if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])>CliffDis)
				{
					PoleJudge[PoleJudgeN][0] = j+1;//下一行中的拟杆件边界点（左边界）的二维像素坐标中的j
					PoleJudge[PoleJudgeN][1] = 1;//1表示正（该点相对于前一点较近），0表示负（该点相对于后一点较近）
					PoleJudgeN++;
				}
				else if ((Frame3D[i][j][2]-Frame3D[i][j+1][2])<-CliffDis)
				{
					PoleJudge[PoleJudgeN][0] = j;//下一行中的拟杆件边界点（右边界）的二维像素坐标中的j
					PoleJudge[PoleJudgeN][1] = 0;//1表示正（该点相对于前一点较近），0表示负（该点相对于后一点较近）
					PoleJudgeN++;
				}
			}
			//////////////////////////////////////////////////////
			if (PoleJudgeN>1)
			{
				int mar=0;//杆件数目
				for (j=0;j<PoleJudgeN-1;j++)
				{
					//对检测到的拟杆件边界点中符合夹持杆件要求的重新整合，将同一根杆件的左右两个拟边界点放在一个数组中统一表示并且
					//存储深度图像中同一行扫描杆件轮廓点
					if (PoleJudge[j][1]==1 && PoleJudge[j+1][1]==0
						&&P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])<80*1.414 
						&& P2Distance(Frame3D[i][PoleJudge[j][0]],Frame3D[i][PoleJudge[j+1][0]])>45)
					{
						mar++;
						//存储该行拟杆件边界点（悬崖点）的三维坐标和二维像素坐标
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
						int tempp=0;//属于该杆件的该行轮廓点数量
						float DisMax = Frame3D[i][PoleJudge[j][0]][2];
						if (Frame3D[i][PoleJudge[j+1][0]][2]>Frame3D[i][PoleJudge[j][0]][2])//确定两个拟杆件边界点中深度值较大者
						{
							DisMax = Frame3D[i][PoleJudge[j+1][0]][2];
						}
						for (q=PoleJudge[j][0];q<PoleJudge[j+1][0];q++)//存储深度图像中同一行扫描杆件轮廓点
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


	//寻找杆件的最近线（脊）
	////////////////////////////////////////////////////////////////////
	float NearestLineP[DetPoleNum][480][3]={0}; //每一行中最近的点（脊点）
	int NearestLinePN[5][480];   //每一行中最近的点在该行中的位置

		for (i=0;i<PoleN;i++)
	{
		for (j=0;j<PoleRowN[i];j++)
		{
			NearestLineP[i][j][2] = 1600;//故意设大些，以保证取得每行中的最近点
			for(int rowPN=0;rowPN<300;rowPN++)
			{
				if (OutlineP[i][j][rowPN][2]==0)
				{
					break;
				}
				//rowPN表示属于杆件的点云个数，这里是要遍历属于杆的所有表面点，找出最近点
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

	//判断脊线是否落在平面内
	///////////////////////////////////////////////////////////
	for (i=0;i<PoleN;i++)
	{
		j=0;
		//计算中间像素行的点到脊点的距离小于15的点的数量
		while(P2Distance(OutlineP[i][PoleRowN[i]/2][j],NearestLineP[i][PoleRowN[i]/2])<15)
		{
			j++;
		}
		int k=j;
		if (k>198)
		{
			k=198;
		}
		//计算中间像素行的点到脊点的距离大于15的点的数量
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
				//AfxMessageBox("平面内");
				InPlane = 0;
				DeteSymbol = false;
			}
			return -1;

		}
	}

	//拟合深度最近点直线
	///////////////////////////////////////////////////////////////
	float NearestLine[DetPoleNum][2][3];
	for (i=0;i<PoleN;i++)
	{
		BoundaryLineFit(PoleRowN[i],NearestLineP[i][0],NearestLine[i][0],NearestLine[i][1]);
	}

	//曲直度（脊的拟合误差）
	for (i=0;i<PoleN;i++)
	{
		poleINFO[i].BendDeg = 0;//“脊”的拟合误差P24
		for (j=0;j<PoleRowN[i];j++)
		{
			poleINFO[i].BendDeg += PtoLineDis(NearestLineP[i][j],NearestLine[i][0],NearestLine[i][1]);// 点到直线的距离函数
		}
		poleINFO[i].BendDeg = poleINFO[i].BendDeg/PoleRowN[i];
	}

	//各个杆件长度
	for (i=0;i<PoleN;i++)
	{
		poleINFO[i].PoleLenght = P2Distance(NearestLineP[i][0],NearestLineP[i][PoleRowN[i]]);
		PoleLenghtTemp[i]=poleINFO[i].PoleLenght;
	}
	//////////////////////////////////////////////////////////////////////////

	//找出边缘距离背脊线所在平面f(x,y)=0（脊平面）最近的点，比较两边边缘最近点距离大小
	//////////////////////////////////////////////////////////////////////////

	//提取夹板之间的判断区域
	///////////////////////////////////////////////////////////////////////////
	int checkhigh[5]={0};//杆长在70mm之内的点云行数
	for (i=0;i<PoleN;i++)
	{
		//杆长在70之内的点云行数
		while (P2Distance(OutlineP[i][(PoleRowN[i]-checkhigh[i])/2][10],OutlineP[i][(PoleRowN[i]+checkhigh[i])/2][10])<70)//
		{
			checkhigh[i] = checkhigh[i]+2;
		}
	}

	float tempdis;
	for (i=0;i<PoleN;i++)
	{
		//在70mm杆长的范围内搜寻拟边界点到该行脊点的最近距离
		for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)//j=0;j<Xcounts;j++)
		{
			//计算起始拟边界点到该行的脊点的距离
			tempdis = P2Distance(P_BoundaryP[i][j][0],NearestLineP[i][j]);
			//记录最近距离
			if (EdgeToBackLineDis[i][0]>tempdis)
			{
				EdgeToBackLineDis[i][0] = tempdis;
			}
			//计算终止拟边界点到该行的脊点的距离
			tempdis = P2Distance(P_BoundaryP[i][j][1],NearestLineP[i][j]);
			//记录最近距离
			if (EdgeToBackLineDis[i][1]>tempdis)
			{
				EdgeToBackLineDis[i][1] = tempdis;
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////
	//使得脊平面方向向量朝向距离大的一边
	/////////////////////////////////////////////////////////////////////////
	int CheckStartFlag[5]; //若为0表示左边大，为1表示右边大
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
		if (CheckStartFlag[i]==0)  //左边大
		{
			for (j=(PoleRowN[i]-checkhigh[i])/2;j<(PoleRowN[i]+checkhigh[i])/2;j++)
			{
				CheckAreaP[i][k][0] = 0;//杆件某一行（k）点云中在脊线左侧或者右侧大于最小距离（EBLDMax[i]-2）的点的个数
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][0]],NearestLineP[i][j])>(EBLDMax[i]-2))
				{
					CheckAreaP[i][k][0] ++;
				}
				CheckAreaP[i][k][1] = 0;//杆件某一行（k）点云中在脊线左侧或者右侧大于距离（11）的点的个数
				while (P2Distance(OutlineP[i][j][CheckAreaP[i][k][1]],NearestLineP[i][j])>11)
				{
					CheckAreaP[i][k][1] ++;
				}
				CheckAreaP[i][k][2] = j;//杆件某一行的行数（）
				k++;
			}
		}
		else if (CheckStartFlag[i]==1) //右边大
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

	//判断方杆圆杆,
	////////////////////////////////////////////////////////////////////////////////
	float CheckPoint[DetPoleNum][5000][3];//某根杆件某行中距离该行脊点在（(EBLDMax[i]-2)，11）范围内的点的坐标
	int CheckTriangleN[DetPoleNum][3];   //检测区域三个点的坐标位置
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

	float CheckPlane[DetPoleNum][2][3]={0};  //检验平面方程
	float CheckLine[DetPoleNum][3]={0};
	int RSJ[DETECT_POLE_NUM_MAX];//HU
	for (i=0;i<PoleN;i++)
	{
		if (CheckPoleStyle(CheckTriangleN[i][2],CheckPoint[i][0])>0.26)
		{
			poleINFO[i].PoleStyle = 1;  //圆杆
			RSJ[i] = 1;
		}
		else
		{
			//poleINFO[i].PoleStyle = -1;  //方杆
			//RSJ[i] = -1;
			//PlaneFitting(CheckTriangleN[i][2],CheckPoint[i][0],CheckPlane[i][0]);
			//CheckLine[i][0] = CheckPlane[i][0][1]*NearestLine[i][0][2] - CheckPlane[i][0][2]*NearestLine[i][0][1];
			//CheckLine[i][1] = CheckPlane[i][0][2]*NearestLine[i][0][0] - CheckPlane[i][0][0]*NearestLine[i][0][2];
			//CheckLine[i][2] = CheckPlane[i][0][0]*NearestLine[i][0][1] - CheckPlane[i][0][1]*NearestLine[i][0][0];
			//VectorUnitization(CheckLine[i]);
		}
	}

	//计算杆件中心线方程和杆件尺寸
	for (i=0;i<PoleN;i++)
	{
		int openglk;
		float PolesCentre[3];
		float PolesTempCentre[3];	
		float tempD;
		float R[2];
		//圆杆
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
			///////////////////////////////寻找杆件轴线的最优化方法GU
			for (j=0;j<11;j++)
			{
				PolesTempCentre[0] = NearestLine[i][1][0]+(10-2*j);//范围：±10
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
							PolesCentre[k] = PolesTempCentre[k];//该处求得杆件的中点位置P25
						}
					}
				}

			}
			for (k=0;k<3;k++)
			{
				poleINFO[i].PoleFunc[1][k] = PolesCentre[k];

				PCentralLineTemp[i][1][k] = PolesCentre[k];//GU
			}
			//杆件半径
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
	//清零静态数组
	memset(Frame3D,0,480*640*3*sizeof(float));
	memset(OutlineP,0,DetPoleNum*480*200*3*sizeof(float));
    t=(cv::getTickCount()-t)/cv::getTickFrequency();
    cout<<"Time: "<<t<<endl;

	return 0;
}

