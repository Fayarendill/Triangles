#ifndef TRIANGLES_PACKER
#define TRIANGLES_PACKER

#include <eigen3/Eigen/Dense>
#include <packing/triangle.h>
#include <vector>
#include <optional>

namespace packing
{
#ifndef MAX_STRIP_DEPTH
#define MAX_STRIP_DEPTH 100
#endif // !MAX_STRIP_DEPTH
	constexpr double c_max_strip_depth = MAX_STRIP_DEPTH;

#ifndef MIN_TRIANGLE_SIZE
#define MIN_TRIANGLE_SIZE 0.01
#endif // !MIN_TRIANGLE_SIZE
	constexpr double c_min_triangle_size = MIN_TRIANGLE_SIZE;

#ifndef MIN_CHAIN_STEP
#define MIN_CHAIN_STEP MIN_TRIANGLE_SIZE
#endif // !MIN_CHAIN_STEP
	constexpr double c_min_chain_step = MIN_CHAIN_STEP;

	class Stripe
	{
	private:
		struct Segment
		{
			std::optional<Eigen::Vector2d> left;
			std::optional<Eigen::Vector2d> right;
		};
		typedef std::vector<Segment> Chain;
	public:
		explicit Stripe(double width) noexcept
			:
			width_{ width },
			current_height_{ 0 }
		{
			fitting_chain_.emplace_back(
				Segment{
					std::nullopt,
					std::make_optional<Eigen::Vector2d>(0,0)
				}
			);

			fitting_chain_.emplace_back(
				Segment{
					std::make_optional<Eigen::Vector2d>(0,0),
					std::make_optional<Eigen::Vector2d>(width,0)
				}
			);

			fitting_chain_.emplace_back(
				Segment{
					std::make_optional<Eigen::Vector2d>(width,0),
					std::nullopt
				}
			);
		}

		~Stripe() {}

		bool pack(const Triangle& new_one);

		double witdh() const;
		double current_height() const;
		std::vector<Triangle> packing() const;
	private:
		static bool better_fitting(const Triangle& a, const Triangle& b); // true if a better than b
	private:
		void fix_fitting_chain_();

		void update_chain_with_triangle_(const Triangle& triangle, const Chain::iterator& pSegment);

		std::optional<Triangle> get_segment_fitting_(
			const Chain::const_iterator& pSegment,
			const Triangle& triangle
		) const;

		bool is_overlaping_(
			const Triangle& triangle,
			const Chain::const_iterator& excluding
		) const;

		bool intersects_(const Segment& a, const Segment& b) const;
	private:
		double current_height_;
		const double width_;
		std::vector<Triangle> packed_;

		Chain fitting_chain_;
	};

} // namespace triangles

#endif // !TRIANGLES_PACKER