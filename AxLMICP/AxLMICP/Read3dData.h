#pragma once
#include <vector>
#include "PointXYZ.h"
#include "utility.h"
using namespace std;
//vector<PointXYZ> * Readdata1(const char* filename);
//vector<PointXYZ> * Readdata(const char* filename,int skipline);
//注意此处数据统一采用右手坐标系
inline vector<PointXYZ> * Readdata(const char* filename)
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
		point.Y=b;
		point.Z=c;
		points->push_back(point);		
		fscanf(in,"%f %f %f",&a,&b,&c);  
	}
	fclose(in);

	printf("Number：%d\n",(int)points->size());
	return points;
}

inline vector<PointXYZ> * ReaddataWithScope(const char* filename,double scope)
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
		point.Y=b;
		point.Z=c;
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

inline vector<PointXYZ> * Readdata(const char* filename,int skipline)
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
		point.Y=b;
		point.Z=c;
		points->push_back(point);		
		fscanf(in,"%f %f %f %d",&a,&b,&c,&d); 
	}
	fclose(in);
	printf("Number：%d\n",(int)points->size());
	return points;
}


inline double * ReadOdometry(const char* filename)
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

inline double * ReadOdometryOneLine(const char* filename)
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
	fscanf(in, "%f %f %f %f %f %f", &a, &b, &c, &d, &e, &f); //a,b,c对应右手坐标系的x,z,y
	pose[0] = a;
	pose[1] = b;
	pose[2] = c;
	r[0] = d;//Radian，*PI/180
	r[1] = e;//Radian，*PI/180
	r[2] = f;//Radian，*PI/180
	EulerToMatrix4(pose, r, Rt);
	fclose(in);

	return Rt;
}
