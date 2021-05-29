#ifndef TRIANGLES_PACKER
#define TRIANGLES_PACKER

#include <eigen3/Eigen/Dense>
#include <triangles/triangle.h>
#include <vector>
#include <optional>

namespace triangles
{

#ifndef MIN_TRIANGLE_SIZE
#define MIN_TRIANGLE_SIZE 0.01
	constexpr double c_min_triangle_size = MIN_TRIANGLE_SIZE;
#endif // !MIN_TRIANGLE_SIZE

	class Strip
	{
	private:
		struct Segment
		{
			std::optional<Eigen::Vector2d> left;
			std::optional<Eigen::Vector2d> right;
		};
		typedef std::vector<Segment> Chain;
	public:
		Strip(double width)
			:
			width_(width),
			fitting_chain_{ {0,0}, {1,0} }
		{}

		~Strip() {}

		void pack(const Triangle& new_one);
	private:
		std::optional<Triangle> get_segment_fitting(
			const Segment& segment,
			const Triangle& triangle
		) const;

		bool is_overlaping(
			const Triangle& triangle
		) const;

		bool intersects(const Segment& a, const Segment& b) const;
	private:
		double width_;
		std::vector<Triangle> packed_;

		Chain fitting_chain_;
	};

} // namespace triangles

#endif // !TRIANGLES_PACKER