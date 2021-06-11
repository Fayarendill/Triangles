#include "triangle.h"

namespace packing
{
	using Vec2d = Eigen::Vector2d;

	constexpr double sqrt_3 = 1.73205080757;
	constexpr double deg_2_rad = EIGEN_PI / 180;

	double Triangle::size() const
	{
		return size_;
	}

	const double& Triangle::angle() const
	{
		return alpha_;
	}

	const Vec2d& Triangle::position() const
	{
		return position_;
	}

	Vec2d Triangle::point_a() const
	{
		Vec2d shift = size_ * Vec2d{ -0.5, -sqrt_3/6 };
		shift = Eigen::Rotation2D(alpha_ * deg_2_rad) * shift;
		return position_ + shift;
	}

	Vec2d Triangle::point_b() const
	{
		Vec2d shift = size_ * Vec2d{ 0, sqrt_3 / 3 };
		shift = Eigen::Rotation2D(alpha_ * deg_2_rad) * shift;
		return position_ + shift;
	}

	Vec2d Triangle::point_c() const
	{
		Vec2d shift = size_ * Vec2d{ 0.5, -sqrt_3 / 6 };
		shift = Eigen::Rotation2D(alpha_ * deg_2_rad) * shift;
		return position_ + shift;
	}

	double& Triangle::angle()
	{
		return alpha_;
	}

	Vec2d& Triangle::position()
	{
		return position_;
	}

	void Triangle::set_point_a(const Vec2d& position, double alpha)
	{
		alpha_ = alpha;
		Vec2d shift = size_ * Vec2d{ 0.5, sqrt_3 / 6 };
		shift = Eigen::Rotation2D(alpha * deg_2_rad) * shift;
		position_ = position + shift;
	}

	void Triangle::set_point_b(const Vec2d& position, double alpha)
	{
		alpha_ = alpha;
		Vec2d shift = size_ * Vec2d{ 0, - sqrt_3 / 3 };
		shift = Eigen::Rotation2D(alpha * deg_2_rad) * shift;
		position_ = position + shift;
	}

	void Triangle::set_point_c(const Vec2d& position, double alpha)
	{
		alpha_ = alpha;
		Vec2d shift = size_ * Vec2d{ -0.5, sqrt_3 / 6 };
		shift = Eigen::Rotation2D(alpha * deg_2_rad) * shift;
		position_ = position + shift;
	}
}


