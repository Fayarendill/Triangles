#include <triangles/triangle.h>
#include <triangles/strip.h>
#include <vector>
#include <optional>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <utility>

namespace triangles
{
	using Vec2d = Eigen::Vector2d;
	using Line2d = Eigen::Hyperplane<double, 2>;

	constexpr double c_sqrt_3 = 1.73205080757;

	bool codirection(const Vec2d& a, const Vec2d& b)
	{
		return !(a.normalized() + b.normalized()).isZero();
	}

	bool Strip::pack(const Triangle& triangle)
	{
		if(std::sqrt(3 * triangle.size() * triangle.size() / 4) > width_)
		{
			return false;
		}

		std::vector<std::pair<Triangle, Chain::iterator>> fittings;
		for(auto pSegment = fitting_chain_.begin(); pSegment != fitting_chain_.end(); ++pSegment)
		{
			if(auto segment_fitting = get_segment_fitting_(pSegment, triangle))
			{
				fittings.emplace_back(std::make_pair(*segment_fitting, pSegment));
			}
		}

		if(fittings.empty())
		{
			return false;
		}
		else
		{
			auto best = std::max_element(
				fittings.cbegin(),
				fittings.cend(),
				[this](const std::pair<Triangle, Chain::iterator>& a, const std::pair<Triangle, Chain::iterator>& b)
				{
					return better_fitting(a.first, b.first);
				}
			);
			packed_.emplace_back(best->first);

			update_chain_with_triangle_(*best);

			fix_fitting_chain_();

			return true;
		}
	}

	bool Strip::better_fitting(const Triangle& a, const Triangle& b)
	{
		if(std::abs(std::abs(a.angle()) - 90) < std::abs(std::abs(a.angle()) - 90))
		{
			return true;
		}
		return false;
	}

	void Strip::fix_fitting_chain_()
	{
		std::vector<Chain::iterator> sorted;
		for(auto pSegment = fitting_chain_.begin(); pSegment != fitting_chain_.end(); ++pSegment)
		{
			if(pSegment->left && pSegment->right)
			{
				sorted.push_back(pSegment);
			}
		}

		std::sort(
			sorted.begin(),
			sorted.end(),
			[](const Chain::iterator& a, const Chain::iterator b)
			{
				return (*a->right - *a->left).norm() < (*b->right - *b->left).norm();
			}
		);

		int max_count = std::max(width_, width_ / c_min_triangle_size);
		auto ppSegment = sorted.begin();
		while(fitting_chain_.size() > max_count && !sorted.empty())
		{
			fitting_chain_.erase(*ppSegment);
			ppSegment = sorted.erase(ppSegment);
		}
	}

	void Strip::update_chain_with_triangle_(const std::pair<Triangle, Chain::iterator>& fitting)
	{
		auto triangle = fitting.first;
		auto pSegment = fitting.second;

		if(pSegment == fitting_chain_.begin())
		{
			auto it = fitting_chain_.emplace(pSegment + 1, Segment{ { triangle.point_c()  }, triangle.point_a() });
			it = fitting_chain_.emplace(it, Segment{ { triangle.point_b() }, { triangle.point_c() } });
			it = fitting_chain_.emplace(it, Segment{ { triangle.point_a() } , { triangle.point_b() } });

			pSegment->right = { triangle.point_a() };
		}
		else if(pSegment == fitting_chain_.end())
		{
			auto it = fitting_chain_.emplace(pSegment, Segment{ { triangle.point_b() }, { triangle.point_a() } });
			it = fitting_chain_.emplace(it, Segment{ { triangle.point_c() } , { triangle.point_b() } });
			it = fitting_chain_.emplace(it, Segment{ pSegment->left, { triangle.point_c() } });

			pSegment->left = { triangle.point_c() };
		}
		if(pSegment != fitting_chain_.begin() && pSegment != fitting_chain_.end())
		{
			if(pSegment->right.value().y() > pSegment->left.value().y())
			{
				auto it = fitting_chain_.emplace(pSegment + 1, Segment{ { triangle.point_c() }, pSegment->right });
				it = fitting_chain_.emplace(it, Segment{ { triangle.point_b() }, { triangle.point_c()  } });
				it = fitting_chain_.emplace(it, Segment{ { triangle.point_a() } , { triangle.point_b() } });
				it = fitting_chain_.emplace(it, Segment{ pSegment->left, { triangle.point_a() } });
			}
			else
			{
				auto it = fitting_chain_.emplace(pSegment, Segment{ { triangle.point_c() }, pSegment->left });
				it = fitting_chain_.emplace(it, Segment{ { triangle.point_b() }, { triangle.point_c()  } });
				it = fitting_chain_.emplace(it, Segment{ { triangle.point_a() }, { triangle.point_b()  } });
				it = fitting_chain_.emplace(it, Segment{ pSegment->right, { triangle.point_a()  } });
			}

			fitting_chain_.erase(pSegment);
		}
	}

	std::optional<Triangle> Strip::get_segment_fitting_(
		const Chain::const_iterator& pSegment,
		const Triangle& triangle
	) const
	{
		auto segment = *pSegment;

		auto maybe_packed = triangle;
		Vec2d shift;
		std::function<bool(const Triangle&)> on_segment;

		if(!segment.left)
		{
			maybe_packed.set_point_c(segment.right.value(), -90);
			shift = Vec2d::UnitY();
			on_segment = [](const Triangle& triangle)
			{
				return true;
			};
		}
		else if(!segment.right)
		{
			maybe_packed.set_point_a(segment.right.value(), 90);
			shift = Vec2d::UnitY();
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
				maybe_packed.set_point_a(
					segment.left.value(),
					std::acos(Vec2d::UnitX().dot(segment_v.normalized())) * 180 / EIGEN_PI
				);
				shift = segment_v.normalized() * c_min_chain_step;
				on_segment = [&segment_v, &segment](const Triangle& triangle)
				{
					return segment_v.norm() > (triangle.point_a() - *segment.left).norm();
				};
			}
			else // or from right
			{
				maybe_packed.set_point_c(
					segment.right.value(),
					-std::acos(Vec2d::UnitX().dot(segment_v.normalized())) * 180 / EIGEN_PI
				);
				shift = -segment_v.normalized() * c_min_chain_step;
				on_segment = [&segment_v, &segment](const Triangle& triangle)
				{
					return (-segment_v).norm() > (triangle.point_c() - *segment.right).norm();
				};
			}
		}

		for(; on_segment(maybe_packed); maybe_packed.position() += shift * c_min_triangle_size)
		{
			if(!is_overlaping_(maybe_packed, pSegment))
			{
				break;
			}
		}

		return is_overlaping_(maybe_packed, pSegment) ? std::nullopt : std::make_optional(maybe_packed);
	}

	bool Strip::is_overlaping_(const Triangle& triangle, const Chain::const_iterator& excluding) const
	{
		for(auto pSegment = fitting_chain_.cbegin(); pSegment != fitting_chain_.cend(); ++pSegment)
		{
			if(pSegment != excluding)
			{
				if(intersects_(*pSegment, Segment{ triangle.point_a(), triangle.point_b() }) ||
					intersects_(*pSegment, Segment{ triangle.point_b(), triangle.point_c() }) ||
					intersects_(*pSegment, Segment{ triangle.point_c(), triangle.point_a() }))
				{
					return true;
				}
			}
		}
		return false;
	}

	bool Strip::intersects_(const Segment& a, const Segment& b) const
	{
		Line2d line_a = Line2d::Through(*a.left, *a.right);
		Line2d line_b = Line2d::Through(*b.left, *b.right);

		if(((*b.left - line_a.projection(*b.left)).normalized() + (*b.right - line_a.projection(*b.right)).normalized()).isZero())
		{
			if(std::abs(((line_a.intersection(line_b) - *a.left).norm() - (line_a.intersection(line_b) - *a.right).norm()))
				== (*a.right - *a.left).norm())
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		else
		{
			return false;
		}
	}
}
