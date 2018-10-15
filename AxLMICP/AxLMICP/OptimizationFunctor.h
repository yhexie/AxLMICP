#pragma once
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include "functor.h"
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
class axlmicp;

#define MatScalar = float;

struct OptimizationFunctor : public Functor < float >
{
	using Functor<float>::values;
	typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorX;
public:
	OptimizationFunctor(int m_data_points, const axlmicp *estimator)
		: Functor<float>(m_data_points), estimator_(estimator)
	{}

	virtual ~OptimizationFunctor() {}
	inline OptimizationFunctor(const OptimizationFunctor &src) :
		Functor<float>(src.m_data_points_), estimator_()
	{
		*this = src;
	}

	int operator () (const VectorX &x, VectorX &fvec) const;

	inline OptimizationFunctor& operator = (const OptimizationFunctor &src)
	{
		Functor<float>::operator=(src);
		estimator_ = src.estimator_;
		return (*this);
	}


	inline void warpPoint(const Eigen::Vector3f & pnt_in, Eigen::Vector3f& pnt_out) const
	{
		pnt_out[0] = static_cast<float> (transform_matrix_(0, 0) * pnt_in[0] + transform_matrix_(0, 1) * pnt_in[1] + transform_matrix_(0, 2) * pnt_in[2] + transform_matrix_(0, 3));
		pnt_out[1] = static_cast<float> (transform_matrix_(1, 0) * pnt_in[0] + transform_matrix_(1, 1) * pnt_in[1] + transform_matrix_(1, 2) * pnt_in[2] + transform_matrix_(1, 3));
		pnt_out[2] = static_cast<float> (transform_matrix_(2, 0) * pnt_in[0] + transform_matrix_(2, 1) * pnt_in[1] + transform_matrix_(2, 2) * pnt_in[2] + transform_matrix_(2, 3));
		//pnt_out.getVector3fMap () = transform_matrix_.topLeftCorner (3, 3) * 
		//                            pnt_in.getVector3fMap () + 
		//                            transform_matrix_.block (0, 3, 3, 1);
		//pnt_out.data[3] = pnt_in.data[3];
	}
public:
	const axlmicp *estimator_;
	Eigen::Matrix4f transform_matrix_;
};

