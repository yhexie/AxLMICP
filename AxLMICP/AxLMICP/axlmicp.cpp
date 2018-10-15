#include "axlmicp.h"
#include "OptimizationFunctor.h"


axlmicp::axlmicp()
{
}

axlmicp::axlmicp(vector<PointXYZ> *model_, vector<PointXYZ> * data_, int itermax_, int critFun_)
	:miniter(5), maxiter(100), critFun(0), m_Threshold(10e-6)
{
	ml_model = new vector<PointXYZ>();//用于重采样
	ml_data = new vector<PointXYZ>();//用于重采样
	dimModel = (int)(model_->size() / 5);
	m_model = MatrixXf::Zero(3, dimModel);
	int nidx = 0;
	for (vector<PointXYZ>::iterator iter = model_->begin(); iter < model_->end() - 5; iter += 5) //iter != modelslist->end(); ++iter)
	{
		PointXYZ tmp = (PointXYZ)*iter;
		m_model(0, nidx) = tmp.X;
		m_model(1, nidx) = tmp.Y;
		m_model(2, nidx) = tmp.Z;
		ml_model->push_back(tmp);
		nidx++;
		//printf("%f %f %f\n",tmp.X,tmp.Y,tmp.Z);
	}

	dimData = (int)(data_->size() / 5);
	m_data = MatrixXf::Zero(3, dimData);
	nidx = 0;
	for (vector<PointXYZ>::iterator iter = data_->begin(); iter < data_->end() - 5; iter += 5) //iter != modelslist->end(); ++iter)
	{
		PointXYZ tmp = (PointXYZ)*iter;
		m_data(0, nidx) = tmp.X;
		m_data(1, nidx) = tmp.Y;
		m_data(2, nidx) = tmp.Z;
		ml_data->push_back(tmp);
		nidx++;
		//printf("%f %f %f\n",tmp.X,tmp.Y,tmp.Z);
	}
	maxiter = itermax_;
	critFun = critFun_;
}

axlmicp::axlmicp(MatrixXf model_, MatrixXf data_, int itermax_, int critFun_)
{
	m_model = model_;
	m_data = data_;
	maxiter = itermax_;
	critFun = critFun_;
}


axlmicp::~axlmicp()
{
}

bool readPt(MatrixXf moedl_, ANNpoint p, int npt, int dim)			// read point (false on EOF)
{
	for (int i = 0; i < dim; i++)
	{
		p[i] = moedl_(i, npt);
	}
	return true;
}

void axlmicp::align()
{
	int M = m_data.rows();//data数据的行数，为2或者3
	int N = m_data.cols();//data数据的列数

	int modelM = m_model.rows();
	int modelN = m_model.cols();

	double	eps = 0;
	MatrixXf Ri;
	MatrixXf Ti;

	int					nPts;					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure

	queryPt = annAllocPt(modelM);					// allocate query point
	dataPts = annAllocPts(modelN, modelM);			// allocate data points
	nnIdx = new ANNidx[1];						// allocate near neigh indices
	dists = new ANNdist[1];						// allocate near neighbor dists

	//modle构建四叉树
	//Tree* m_tree=build_kdtree(raw_model,modelN,modelM,index,modelN,0);

	nPts = 0;
	//将m_model转换为ANNpointArray
	while (nPts < modelN && readPt(m_model, dataPts[nPts], nPts, modelM))
	{
		nPts++;
	}

	kdTree = new ANNkd_tree(dataPts, nPts, modelM);

	double oldres = 0;
	double	res = 9e99;
	//迭代的主体循环
	for (int iter = 1; iter < maxiter; iter++)
	{
		printf("第%d次迭代\n", (int)iter);
		oldres = res;
		int * closepoint = new int[N];
		double * distance = new double[N];
		//查询data中每个点的近邻
		//run_queries(m_tree->rootptr,raw_data,N,3,closepoint,distance,RETURN_INDEX );
		for (int i = 0; i < N; i++)
		{
			readPt(m_data, queryPt, i, M);
			kdTree->annkSearch(queryPt, 1, nnIdx, dists, eps);
			//要不要增加最远距离的判断，如果判断则对应点数组不为N，小于N？
			closepoint[i] = nnIdx[0];
			distance[i] = dists[0];
		}

		MatrixXf matrixDis(1, N);
		double sumResidual = 0;
		for (int i = 0; i < N; i++)
		{
			matrixDis(0, i) = distance[i];
			sumResidual += distance[i] * distance[i];
		}
		//提取对应点
		consporsone = MatrixXf::Zero(M, N);
		for (int i = 0; i < N; i++)
		{
			for (int j = 0; j < M; j++)
			{
				consporsone(j, i) = m_model(j, closepoint[i]);
			}
		}


		res = sumResidual / N;
		MatrixXf med = m_data.rowwise().sum() / N;

		/*mem=mean(model(:,vi),2);*/
		MatrixXf mem = consporsone.rowwise().sum() / consporsone.cols();

		VectorX x(6);
		x.setConstant(6, 0);
		OptimizationFunctor functor(static_cast<int> (N), this);
		Eigen::NumericalDiff<OptimizationFunctor> num_diff(functor);
		//Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm (num_diff);
		Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, float> lm(num_diff);
		int info = lm.minimize(x);

		printf("LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm.fvec.norm());
		printf("Final solution: [%f", x[0]);
		for (int i = 1; i < 6; ++i)
			printf(" %f", x[i]);
		printf("]\n");
		Eigen::Matrix4f transform_matrix_;
		transform_matrix_.setZero();
		transform_matrix_(0, 3) = x[0];
		transform_matrix_(1, 3) = x[1];
		transform_matrix_(2, 3) = x[2];
		transform_matrix_(3, 3) = 1;

		// Compute w from the unit quaternion
		Eigen::Quaternion<float> q(0, x[3], x[4], x[5]);
		q.w() = static_cast<float> (sqrt(1 - q.dot(q)));
		q.normalize();
		transform_matrix_.topLeftCorner(3, 3) = q.toRotationMatrix();

		/*warp_point_->setParam(x);
		transformation_matrix = warp_point_->getTransform();
		*/
		Ri = transform_matrix_.topLeftCorner(3, 3);

		m_data = Ri*m_data;//+Ti;                       // Apply transformation
		for (int i = 0; i < N; i++)
		{
			m_data(0, i) += x[0];
			m_data(1, i) += x[1];
			m_data(2, i) += x[2];
		}
		free(closepoint);
		free(distance);
		if (iter >= miniter)
		{
			if (abs(oldres - res) < m_Threshold)
				break;
		}

	}
}

void axlmicp::SaveAlignData(string filename)
{
	string sub = filename.substr(0, filename.find_last_of("."));
	string spendix = filename.substr(filename.find_last_of("."));
	string filename1 = sub + "_1" + spendix;
	const char * file = filename1.c_str();
	FILE* in = fopen(file, "w");
	if (in == NULL)
	{
		printf("missing file");
		return;
	}
	for (int i = 0; i < m_data.cols(); i++)
	{
		fprintf(in, "%f %f %f\n", m_data(0, i), m_data(1, i), m_data(2, i));
	}
	fclose(in);
}