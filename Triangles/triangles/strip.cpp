#include <triangles/triangle.h>
#include <triangles/strip.h>
#include <vector>
#include <optional>
#include <eigen3/Eigen/Dense>

namespace triangles
{
	void Strip::pack(const Triangle& triangle)
	{

	}

	std::optional<Triangle> Strip::get_segment_fitting(
		const Strip::Segment& segment,
		const Triangle& triangle
	) const
	{
		auto maybe_packed = triangle;
		Eigen::Vector2d shift;
		std::function<bool(const Triangle&)> on_segment;

		if(!segment.left)
		{
			maybe_packed.position() = segment.right.value();
			shift = Eigen::Vector2d::UnitY();
			on_segment = [](const Triangle& triangle)
			{
				return true;
			};
		}
		else if(!segment.right)
		{
			maybe_packed.position() = segment.left.value();
			shift = Eigen::Vector2d::UnitY();
			on_segment = [](const Triangle& triangle)
			{
				return true;
			};
		}
		else if(segment.left && segment.right)
		{
			auto segment_v = *segment.right - *segment.left;
			if(segment.right.value().y() > segment.left.value().y()) // moving from left segment point
			{
				maybe_packed.set_point_a(segment.left.value());
				shift = segment_v.normalized();
				on_segment = [&segment_v, &segment](const Triangle& triangle)
				{
					return segment_v.norm() > (triangle.point_a() - *segment.left).norm();
				};
			}
			else // or from right
			{
				maybe_packed.set_point_c(segment.right.value());
				shift = -segment_v.normalized();
				on_segment = [&segment_v, &segment](const Triangle& triangle)
				{
					return (-segment_v).norm() > (triangle.point_c() - *segment.right).norm();
				};
			}
		}

		for(; on_segment(maybe_packed); maybe_packed.position() += shift * c_min_triangle_size)
		{
			if(!is_overlaping(maybe_packed))
			{
				break;
			}
		}

		return is_overlaping(maybe_packed) ? std::nullopt : std::make_optional(maybe_packed);
	}

	bool Strip::is_overlaping(const Triangle& triangle) const
	{
		for(const auto& segment : fitting_chain_)
		{
			if(intersects(segment, Segment{ triangle.point_a(), triangle.point_b() }) ||
				intersects(segment, Segment{ triangle.point_b(), triangle.point_c() }) ||
				intersects(segment, Segment{ triangle.point_c(), triangle.point_a() }))
			{
				return true;
			}
		}
		return false;
	}

	bool Strip::intersects(const Segment& a, const Segment& b) const
	{
		return false;
	}
}
