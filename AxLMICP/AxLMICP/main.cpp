#include <vector>
#include "PointXYZ.h"
#include "axlmicp.h"
#include <Eigen/Dense>
#include "Read3dDataLeft.h"
#include "Read3dData.h"
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
int main()
{
#pragma region params
	int startFrame = 1;//∆ º÷°
	int endFrame = 122;//÷’÷π÷°
	double scope = 100000;//…®√Ë…œœﬁæ‡¿Îcm
	double rejectDistance = 15;//m
	string path = "data";
	int maxiter = 800;
#pragma endregion params
	string posefilename = path + "\\finalpose.txt";
	const char * posefile = posefilename.c_str();
	FILE* inpose = fopen(posefile, "w");
	if (inpose == NULL)
	{
		printf("missing file");
		return 0;
	}
	string filename = path + "\\laser_1.txt";
	string filename1 = path + "\\laser_2.txt";
	vector<PointXYZ> * modelslist = ReaddataWithScope(filename.c_str(), scope);//…®√Ë∞Îæ∂¥Û∏≈3000cm
	vector<PointXYZ> * dataslist = ReaddataWithScope(filename1.c_str(), scope);

	axlmicp icp(modelslist, dataslist, maxiter, 4);
	icp.align();
	icp.SaveAlignData(filename1);
	system("pause");
	//icp.SetRejectDistance(false, rejectDistance);
}