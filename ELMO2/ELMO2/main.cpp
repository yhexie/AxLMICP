// 示例：y = 10*(x0+3)^2 + (x1-5)^2
#include "math.h"
#include "iostream"
#include "vector"
#include "list"

using namespace std;

#include "Eigen/Dense"
#include "Eigen/Core"
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

using namespace Eigen;
// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
	typedef _Scalar Scalar;
	enum {
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }

};
struct my_functor : Functor < double >
{
	// 输出个数必须大于输入个数, 故用2不用1;
	my_functor(void) : Functor<double>(2, 2) {}
	int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
	{
		// Implement y = 10*(x0+3)^2 + (x1-5)^2
		fvec(0) = 10.0*pow(x(0) + 3.0, 2) + pow(x(1) - 5.0, 2);
		fvec(1) = 0;

		return 0;
	}
};
int main(int argc, char *argv[])
{
	Eigen::VectorXd x(2);
	x(0) = 1.0;
	x(1) = 3.0;

	my_functor functor;
	Eigen::NumericalDiff<my_functor> numDiff(functor);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, double> lm(numDiff);

	Eigen::VectorXd y(2);
	functor.operator()(x, y);

	std::cout << "x first input: \n" << x << std::endl;
	std::cout << "y first outpout: \n" << y << std::endl;
	lm.parameters.maxfev = 1000;
	lm.parameters.xtol = 1.0e-8;
	int iRet = lm.minimize(x);
	std::cout << "迭代次数：\n" << lm.iter << std::endl;
	std::cout << "计算标志：\n" << iRet << std::endl;
	std::cout << "x finnal: \n" << x << std::endl;
	functor.operator()(x, y);
	std::cout << "y outpout((minimized): \n" << y << std::endl;
	system("pause");
	return 0;
}
