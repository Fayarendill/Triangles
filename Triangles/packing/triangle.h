#ifndef TRIANGLES_TRIANGLE
#define TRIANGLES_TRIANGLE

#include <eigen3/Eigen/Dense>

namespace packing
{
	///
	/// Equilateral triangle class
	/// @member size: side length
	///
	class Triangle
	{
	public:
		explicit Triangle(double size)
			: size_(size)
			, alpha_(0.0)
		{
		}

		~Triangle()
		{
		}

		double size() const;

		const double& angle() const;
		///
		/// @returns: position of triangle's center
		///
		const Eigen::Vector2d& position() const;
		Eigen::Vector2d point_a() const;
		Eigen::Vector2d point_b() const;
		Eigen::Vector2d point_c() const;

		double& angle();
		///
		/// @returns: position of triangle's center
		///
		Eigen::Vector2d& position();
		void set_point_a(const Eigen::Vector2d&, double alpha); // left most vertex
		void set_point_b(const Eigen::Vector2d&, double alpha); // middle vertex
		void set_point_c(const Eigen::Vector2d&, double alpha); // right most vertex
	private:
		const double size_;
		double alpha_;
		Eigen::Vector2d position_;
	};

} // namespace triangles

#endif // !TRIANGLES_TRIANGLE


