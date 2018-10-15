#pragma once
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
	typedef _Scalar Scalar;
	enum
	{
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<_Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<_Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<_Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	/** \brief Empty Construtor. */
	Functor() : m_data_points_(ValuesAtCompileTime) {}

	/** \brief Constructor
	* \param[in] m_data_points number of data points to evaluate.
	*/
	Functor(int m_data_points) : m_data_points_(m_data_points) {}

	/** \brief Destructor. */
	virtual ~Functor() {}

	/** \brief Get the number of values. */
	int values() const 
	{ 
		return (m_data_points_);
	}

protected:
	int m_data_points_;
};


