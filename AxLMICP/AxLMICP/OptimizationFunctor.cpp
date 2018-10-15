#include "OptimizationFunctor.h"
#include "axlmicp.h"

int OptimizationFunctor::operator()(const VectorX &p, VectorX &fvec) const
{
	const MatrixXf & src_points = estimator_->m_data;
	const MatrixXf & tgt_points = estimator_->consporsone;
	Eigen::Matrix4f transform_matrix_;
	// Initialize the warp function with the given parameters
	transform_matrix_.setZero();
	transform_matrix_(0, 3) = p[0];
	transform_matrix_(1, 3) = p[1];
	transform_matrix_(2, 3) = p[2];
	transform_matrix_(3, 3) = 1;

	// Compute w from the unit quaternion
	Eigen::Quaternion<float> q(0, p[3], p[4], p[5]);
	q.w() = static_cast<float> (sqrt(1 - q.dot(q)));
	q.normalize();
	transform_matrix_.topLeftCorner(3, 3) = q.toRotationMatrix();

	// Transform each source point and compute its distance to the corresponding target point

	for (int i = 0; i < values(); ++i)
	{
		const Vector3f  p_src(src_points(0, i), src_points(1, i), src_points(2, i));
		const Vector3f  p_tgt(tgt_points(0, i), tgt_points(1, i), tgt_points(2, i));

		// Transform the source point based on the current warp parameters
		Vector4f p_src_warped;
		p_src_warped[0] = static_cast<float> (transform_matrix_(0, 0) * p_src[0] + transform_matrix_(0, 1) * p_src[1] + transform_matrix_(0, 2) * p_src[2] + transform_matrix_(0, 3));
		p_src_warped[1] = static_cast<float> (transform_matrix_(1, 0) * p_src[0] + transform_matrix_(1, 1) * p_src[1] + transform_matrix_(1, 2) * p_src[2] + transform_matrix_(1, 3));
		p_src_warped[2] = static_cast<float> (transform_matrix_(2, 0) * p_src[0] + transform_matrix_(2, 1) * p_src[1] + transform_matrix_(2, 2) * p_src[2] + transform_matrix_(2, 3));
		Vector4f s(p_src_warped[0], p_src_warped[1], p_src_warped[2], 0);
		Vector4f t(p_tgt[0], p_tgt[1], p_tgt[2], 0);
		// Estimate the distance (cost function)
		fvec[i] = ((s - t).norm());
	}
	return (0);
}

