#pragma once
#include <vector>
#include "PointXYZ.h"
#include "utility.h"
using namespace std;

//因为数据集使用左手坐标系，所以此处多写了一个类用于读取数据
//数据集：5_building of University of Osnabrück\dat_et4\dat_et4
inline vector<PointXYZ> * ReaddataLeft(const char* filename)
{
	FILE* in=fopen(filename,"r");
	if(in==NULL)
	{
		printf("missing file");
		return NULL;
	}
	double tmp=0;
	char buff[255] = {};
	float a=0.0f,b=0.0f,c=0.0f;
	vector<PointXYZ> *points=new vector<PointXYZ>();
	int i=0;

	fscanf(in,"%f %f %f",&a,&b,&c); 
	while(feof(in)==0) /*判断是否文件尾，不是则循环*/
	{
		i++;	 
		PointXYZ point;
		point.X=a;
		point.Y=c;
		point.Z=b;
		points->push_back(point);		
		fscanf(in,"%f %f %f",&a,&b,&c);  
	}
	fclose(in);

	printf("Number：%d\n",(int)points->size());
	return points;
}

inline vector<PointXYZ> * ReaddataWithScopeLeft(const char* filename,double scope)
{
	FILE* in=fopen(filename,"r");
	if(in==NULL)
	{
		printf("missing file");
		return NULL;
	}
	double tmp=0;
	char buff[255] = {};
	float a=0.0f,b=0.0f,c=0.0f;
	vector<PointXYZ> *points=new vector<PointXYZ>();
	char* str;
	int i=0;

	fscanf(in,"%f %f %f",&a,&b,&c); 
	while(feof(in)==0) /*判断是否文件尾，不是则循环*/
	{
		i++;	 
		PointXYZ point;
		point.X=a;
		point.Y=c;
		point.Z=b;
		double dd=a*a+b*b+c*c;
		dd=sqrt(dd);
		if (dd<scope)
		{
			points->push_back(point);		
		}		
		fscanf(in,"%f %f %f",&a,&b,&c);  
	}
	fclose(in);

	printf("Number：%d\n",(int)points->size());
	return points;
}

inline vector<PointXYZ> * ReaddataLeft(const char* filename,int skipline)
{
	FILE* in=fopen(filename,"r");
	if(in==NULL)
	{
		printf("missing file");
		return NULL;
	}
	double tmp=0;
	char buff[255] = {};
	float a=0.0f,b=0.0f,c=0.0f;
	int d;
	vector<PointXYZ> *points=new vector<PointXYZ>();
	char* str;
	int  m=0,n=0,i=0;
	char* s=new char[4];
	for (int idxSkip=0;i<skipline;i++)
	{
		fscanf(in,"%d %s %d",&m,s,&n);
	}
	
	fscanf(in,"%f %f %f %d",&a,&b,&c,&d); 
	while(feof(in)==0) /*判断是否文件尾，不是则循环*/
	{
		i++;	 
		PointXYZ point;
		point.X=a;
		point.Y=c;
		point.Z=b;
		points->push_back(point);		
		fscanf(in,"%f %f %f %d",&a,&b,&c,&d); 
	}
	fclose(in);
	printf("Number：%d\n",(int)points->size());
	return points;
}


inline double * ReadOdometryLeft(const char* filename)
{
	FILE* in = fopen(filename, "r");
	if (in == NULL)
	{
		printf("missing file");
		return NULL;
	}
	double *pose = new double[3];
	double *r = new double[3];
	double *Rt = new double[16];
	int i = 0;
	float a, b, c = 0.0f;
	float d, e, f = 0.0f;
	fscanf(in, "%f %f %f", &a, &b, &c); //a,b,c对应右手坐标系的x,z,y
	fscanf(in, "%f %f %f", &d, &e, &f); //d,e,f对应右手坐标系的x,z,y
	pose[0] = a;
	pose[1] = c;
	pose[2] = b;
	r[0] = d*M_PI / 180;
	r[1] = f*M_PI / 180;
	r[2] = e*M_PI / 180;
	EulerToMatrix4(pose, r, Rt);
	fclose(in);

	return Rt;
}
