#include <packing/triangle.h>
#include <packing/stripe.h>
#include <vector>
#include <optional>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <utility>
#include <iostream>
#include <sstream>

#include <boost/lexical_cast.hpp>

namespace packing
{
	using Vec2d = Eigen::Vector2d;
	using Line2d = Eigen::Hyperplane<double, 2>;


	namespace
	{
		bool codirection(const Vec2d& a, const Vec2d& b, double prec = c_default_prec)
		{
			return !(a.normalized() + b.normalized()).isZero(prec);
		}

		std::ostream& operator<<(std::ostream& o, const Vec2d& vec)
		{
			o
				<< "["
				<< vec.x()
				<< ", "
				<< vec.y()
				<< "]";
			return o;
		}

		template<typename T>
		std::ostream& operator<<(std::ostream& o, const std::optional<T>& vec)
		{
			if(vec)
			{
				o << *vec;
			}
			else
			{
				o << "none";
			}
			return o;
		}

		std::ostream& operator<<(std::ostream& o, const Stripe::Segment& segment)
		{
			o
				<< "("
				<< segment.left()
				<< ", "
				<< segment.right()
				<< ")";
			return o;
		}

		std::ostream& operator<<(std::ostream& o, const Stripe::Chain& chain)
		{
			std::stringstream debug_str;
			for(auto pSegment = chain.begin(); pSegment != chain.end() - 1; ++pSegment)
			{
				o << *pSegment << " -> ";
			}

			if(!chain.empty())
			{
				o << *(chain.end() - 1);
			}
			return o;
		}

		std::ostream& operator<<(std::ostream& o, const Triangle& triangle)
		{
			o
				<< "center:"
				<< triangle.position()
				<< " point a:"
				<< triangle.point_a()
				<< " point b:"
				<< triangle.point_b()
				<< " point c:"
				<< triangle.point_c();
			return o;
		}

		std::ostream& operator<<(std::ostream& o, const std::pair<Triangle, Stripe::Chain::iterator>& fitting)
		{
			o
				<< "segment: " << *fitting.second
				<< " pose: " << fitting.first;
			return o;
		}

		bool is_approx(double x, double y, double prec = c_default_prec)
		{
			//return Eigen::Matrix<double, 1, 1>{ x }.isApprox(Eigen::Matrix<double, 1, 1>{ y });
			//return Eigen::Matrix<double, 1, 1>{ x }.isApproxToConstant(y, 1e-10);
			return std::abs(x - y) < prec;
		}
	}


	bool Stripe::pack(const Triangle& triangle)
	{
		std::cout
			<< "Stripe::pack()"
			<< " Triangle is " << triangle
			<< std::endl;

		std::cout
			<< "Stripe::pack()"
			<< " Chain is " << fitting_chain_
			<< std::endl;

		if(c_sqrt_3 * triangle.size() / 2 > width_)
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
			auto best = std::min_element(
				fittings.cbegin(),
				fittings.cend(),
				[this](const std::pair<Triangle, Chain::iterator>& a, const std::pair<Triangle, Chain::iterator>& b)
				{
					return better_fitting(a.first, b.first);
				}
			);

			std::cout
				<< "Stripe::pack()"
				<< " Fitting is " << *best
				<< std::endl;

			packed_.emplace_back(best->first);

			update_chain_with_triangle_(best->first, best->second);

			std::cout
				<< "Stripe::pack()"
				<< " Updated chain is " << fitting_chain_
				<< std::endl;

			//fix_fitting_chain_();

			return true;
		}
	}

	double Stripe::width() const
	{
		return width_;
	}

	double Stripe::current_height() const
	{
		auto current_height = 0.0;
		for(auto& segment : fitting_chain_)
		{
			if(segment.left() && segment.left()->y() > current_height)
			{
				current_height = segment.left()->y();
			}
			if(segment.right() && segment.right()->y() > current_height)
			{
				current_height = segment.right()->y();
			}
		}
		return current_height;
	}

	std::vector<Triangle> Stripe::packing() const
	{
		return packed_;
	}

	std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> Stripe::fitting_chain() const
	{
		std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>  chain;
		for(auto& segment : fitting_chain_)
		{
			chain.push_back(std::make_pair(
				segment.left() ? segment.left().value() : Eigen::Vector2d{ segment.right().value().x(), segment.right().value().y() + width_ },
				segment.right() ? segment.right().value() : Eigen::Vector2d{ segment.left().value().x(), segment.left().value().y() + width_ }
			));
		}
		return chain;
	}

	bool Stripe::better_fitting(const Triangle& a, const Triangle& b)
	{
		//if(a.position().y() == b.position().y())
		//{
		//	if(std::abs(std::abs(a.angle()) - 90) < std::abs(std::abs(a.angle()) - 90))
		//	{
		//		return true;
		//	}
		//	else
		//	{
		//		return false;
		//	}
		//}
		//else if(a.position().y() < b.position().y())
		//{
		//	return true;
		//}
		//else
		//{
		//	return false;
		//}
		return a.position().y() < b.position().y();
	}

	void Stripe::fix_fitting_chain_(Chain::iterator& last_inserted)
	{
		Chain::iterator pSegment, first_inserted;
		std::cout
			<< "Stripe::fix_fitting_chain_()"
			<< " Chain is " << fitting_chain_
			<< std::endl;
		for(pSegment = last_inserted + 1; pSegment != fitting_chain_.end(); )
		{
			if(pSegment->right() && pSegment->left())
			{
				if(last_inserted->lies_on(*pSegment->left()) && last_inserted->lies_on(*pSegment->right()))
				{
					if(pSegment->lies_on(*last_inserted->left()) && pSegment->lies_on(*last_inserted->right())) // 1.1
					{
						pSegment = last_inserted = fitting_chain_.erase(last_inserted, pSegment + 1);
						std::cout
							<< "Stripe::fix_fitting_chain_()"
							<< " Fixing 1.1, chain is " << fitting_chain_
							<< std::endl;
						break;
					}
					else //if(codirection(pSegment->vector(), *last_inserted->left() - *pSegment->right())) //1.2
					{
						last_inserted->right() = pSegment->right();
						last_inserted = fitting_chain_.erase(last_inserted + 1, pSegment + 1);
						pSegment = last_inserted + 1;
						std::cout
							<< "Stripe::fix_fitting_chain_()"
							<< " Fixing 1.2, chain is " << fitting_chain_
							<< std::endl;
						continue;
					}
				}
				else if(pSegment->lies_on(*last_inserted->left()) && pSegment->lies_on(*last_inserted->right()) /*&&
					codirection(last_inserted->vector(), *last_inserted->left() - *pSegment->right())*/)
				{
					if(last_inserted->lies_on(*pSegment->right())) //1.3
					{
						pSegment = fitting_chain_.erase(last_inserted, pSegment + 1);
						std::cout
							<< "Stripe::fix_fitting_chain_()"
							<< " Fixing 1.3, chain is " << fitting_chain_
							<< std::endl;
						break;
					}
					else //1.4
					{
						pSegment->left() = last_inserted->left();
						pSegment = fitting_chain_.erase(last_inserted, pSegment);
						std::cout
							<< "Stripe::fix_fitting_chain_()"
							<< " Fixing 1.4, chain is " << fitting_chain_
							<< std::endl;
						break;
					}
				}
				else if(last_inserted->lies_on(*pSegment->left()) && pSegment->lies_on(*last_inserted->left())) //1.5
				{
					pSegment->left() = last_inserted->left();
					pSegment = fitting_chain_.erase(last_inserted, pSegment);
					std::cout
						<< "Stripe::fix_fitting_chain_()"
						<< " Fixing 1.5, chain is " << fitting_chain_
						<< std::endl;
					break;
				}
				//else if(last_inserted->lies_on(*pSegment->left())) //1.6
				//{
				//	last_inserted->right() = pSegment->left();
				//	pSegment = fitting_chain_.erase(last_inserted + 1, pSegment);
				//	std::cout
				//		<< "Stripe::fix_fitting_chain_()"
				//		<< " Fixing 1.6, chain is " << fitting_chain_
				//		<< std::endl;
				//	continue;
				//}
				else if(last_inserted->lies_on(*pSegment->right(), 0.005)) //1.7
				{
					last_inserted->right() = pSegment->right();
					pSegment = fitting_chain_.erase(last_inserted + 1, pSegment + 1);
					std::cout
						<< "Stripe::fix_fitting_chain_()"
						<< " Fixing 1.7, chain is " << fitting_chain_
						<< std::endl;
					continue;
				}
				else if(pSegment->lies_on(*last_inserted->left(), 0.005)) //1.8
				{
					pSegment->left() = last_inserted->left();
					pSegment = fitting_chain_.erase(last_inserted + 1, pSegment + 1);
					std::cout
						<< "Stripe::fix_fitting_chain_()"
						<< " Fixing 1.8, chain is " << fitting_chain_
						<< std::endl;
					break;
				}
				//else if(pSegment->lies_on(*last_inserted->right())) //1.9
				//{
				//	std::cout
				//		<< "Stripe::fix_fitting_chain_()"
				//		<< " Fixing 1.9, chain is " << fitting_chain_
				//		<< std::endl;
				//	break;
				//}
				else
				{
					++pSegment;
				}
			}
			else
			{
				++pSegment;
			}
		}

		//bool new_found = false;
		//while(!new_found || pSegment->is_new())
		//{
		//	if(pSegment->is_new())
		//	{
		//		new_found = true;
		//	}
		//	first_inserted = pSegment--;
		//}

		//for(; pSegment != fitting_chain_.cbegin(); --pSegment)
		//{
		//	if(pSegment->right() && pSegment->left())
		//	{
		//		if(last_inserted->lies_on(*pSegment->left()) && last_inserted->lies_on(*pSegment->right()))
		//		{
		//			if(pSegment->lies_on(*last_inserted->left()))
		//			{
		//				pSegment = last_inserted = fitting_chain_.erase(last_inserted, pSegment);
		//				break;
		//			}
		//			else //if(codirection(pSegment->vector(), *last_inserted->left() - *pSegment->right()))
		//			{
		//				last_inserted->right() = pSegment->right();
		//				last_inserted = fitting_chain_.erase(last_inserted + 1, pSegment);
		//				pSegment = last_inserted + 1;
		//				continue;
		//			}
		//		}
		//		else if(pSegment->lies_on(*last_inserted->left()) && pSegment->lies_on(*last_inserted->right()) /*&&
		//			codirection(last_inserted->vector(), *last_inserted->left() - *pSegment->right())*/)
		//		{
		//			if(last_inserted->lies_on(*pSegment->right()))
		//			{
		//				pSegment = fitting_chain_.erase(last_inserted, pSegment);
		//				break;
		//			}
		//			else
		//			{
		//				pSegment->left() = last_inserted->left();
		//				pSegment = fitting_chain_.erase(last_inserted, pSegment - 1);
		//				break;
		//			}
		//		}
		//		else if(last_inserted->lies_on(*pSegment->left()) && pSegment->lies_on(*last_inserted->left()))
		//		{
		//			pSegment->left() = last_inserted->left();
		//			pSegment = fitting_chain_.erase(last_inserted, pSegment - 1);
		//			break;
		//		}
		//		else
		//		{
		//			++pSegment;
		//		}
		//	}
		//	else
		//	{
		//		++pSegment;
		//	}
		//}
	}

	void Stripe::update_chain_with_triangle_(const Triangle& triangle, const Chain::iterator& pSegment)
	{
		Chain::iterator it;

		if(!pSegment->left())
		{
			auto segment_right = pSegment->right();
			pSegment->right() = { triangle.point_a() };

			if(!segment_right->isApprox(triangle.point_c()))
			{
				it = fitting_chain_.emplace(pSegment + 1, Segment{ { triangle.point_c()  }, segment_right, false });
			}
			it = fitting_chain_.emplace(
				segment_right->isApprox(triangle.point_c()) ? pSegment + 1 : it, 
				Segment{ { triangle.point_b() }, { triangle.point_c() } });
			it = fitting_chain_.emplace(it, Segment{ { triangle.point_a() } , { triangle.point_b() } });

			it += 1l;
		}
		else if(!pSegment->right())
		{
			auto segment_left = pSegment->left();
			pSegment->left() = { triangle.point_c() };

			it = fitting_chain_.emplace(pSegment, Segment{ { triangle.point_b() }, { triangle.point_c() } });
			it = fitting_chain_.emplace(it, Segment{ { triangle.point_a() } , { triangle.point_b() } });
			if(!segment_left->isApprox(triangle.point_a()))
			{
				it = fitting_chain_.emplace(it, Segment{ pSegment->left(), { triangle.point_a() }, false });
			}

			it += 1l + (!segment_left->isApprox(triangle.point_a()) ? 1l : 0l);
		}
		else
		{
			auto segment_right = pSegment->right();
			auto segment_left = pSegment->left();
			it = fitting_chain_.erase(pSegment);

			if(!segment_right->isApprox(triangle.point_c()))
			{
				it = fitting_chain_.emplace(it, Segment{ { triangle.point_c() }, segment_right, false });
			}
			it = fitting_chain_.emplace(it, Segment{ { triangle.point_b() }, { triangle.point_c()  } });
			it = fitting_chain_.emplace(it, Segment{ { triangle.point_a() } , { triangle.point_b() } });
			if(!segment_left->isApprox(triangle.point_a()))
			{
				it = fitting_chain_.emplace(it, Segment{ segment_left, { triangle.point_a() }, false });
			}

			it += 1l + (!segment_left->isApprox(triangle.point_a()) ? 1l : 0l);
		}

		fix_fitting_chain_(it);
	}

	std::optional<Triangle> Stripe::get_segment_fitting_(
		const Chain::const_iterator& pSegment,
		const Triangle& triangle
	) const
	{
		auto segment = *pSegment;

		auto maybe_packed = triangle;
		Vec2d shift;
		std::function<bool(const Triangle&)> on_segment;

		if(!segment.left())
		{
			maybe_packed.set_point_c(segment.right().value(), -90);
			shift = Vec2d::UnitY();
			on_segment = [](const Triangle& triangle)
			{
				return true;
			};
		}
		else if(!segment.right())
		{
			maybe_packed.set_point_a(segment.left().value(), 90);
			shift = Vec2d::UnitY();
			on_segment = [](const Triangle& triangle)
			{
				return true;
			};
		}
		else if(segment.left() && segment.right())
		{
			auto segment_v = segment.vector();
			if(segment.right().value().y() > segment.left().value().y()) // moving from left segment point
			{
				maybe_packed.set_point_a(
					segment.left().value(),
					std::acos(Vec2d::UnitX().dot(segment_v.normalized())) * 180 / EIGEN_PI
				);
				shift = segment_v.normalized() * c_min_chain_step;
				on_segment = [&segment_v, &segment](const Triangle& triangle)
				{
					//return segment_v.norm() > (triangle.point_a() - *segment.left).norm();
					return !codirection(triangle.point_a() - *segment.left(), triangle.point_a() - *segment.right());
				};
			}
			else // or from right
			{
				maybe_packed.set_point_c(
					segment.right().value(),
					-std::acos(Vec2d::UnitX().dot(segment_v.normalized())) * 180 / EIGEN_PI
				);
				shift = -segment_v.normalized() * c_min_chain_step;
				on_segment = [&segment_v, &segment](const Triangle& triangle)
				{
					//return (-segment_v).norm() > (triangle.point_c() - *segment.right).norm();
					return !codirection(triangle.point_c() - *segment.left(), triangle.point_c() - *segment.right());
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

	bool Stripe::is_overlaping_(const Triangle& triangle, const Chain::const_iterator& excluding) const
	{
		for(auto pSegment = fitting_chain_.cbegin(); pSegment != fitting_chain_.cend(); ++pSegment)
		{
			if(pSegment != excluding)
			{
				if(pSegment->left() && pSegment->right())
				{
					if(intersects_(*pSegment, Segment{ triangle.point_a(), triangle.point_b() }) ||
						intersects_(*pSegment, Segment{ triangle.point_b(), triangle.point_c() }) ||
						intersects_(*pSegment, Segment{ triangle.point_c(), triangle.point_a() }))
					{
						return true;
					}
				}
				else
				{
					Segment fictive_segment;
					if(pSegment->left())
					{
						fictive_segment = Segment{ pSegment->left(), std::make_optional<Vec2d>(width_, c_max_strip_depth) };
					}
					else if(pSegment->right())
					{
						fictive_segment = Segment{ std::make_optional<Vec2d>(0, c_max_strip_depth), pSegment->right() };
					}

					if(intersects_(fictive_segment, Segment{ triangle.point_a(), triangle.point_b() }) ||
						intersects_(fictive_segment, Segment{ triangle.point_b(), triangle.point_c() }) ||
						intersects_(fictive_segment, Segment{ triangle.point_c(), triangle.point_a() }))
					{
						return true;
					}
				}
			}
		}
		return false;
	}

	bool Stripe::intersects_(const Segment& a, const Segment& b) const
	{
		Line2d line_a = Line2d::Through(*a.left(), *a.right());
		Line2d line_b = Line2d::Through(*b.left(), *b.right());

		auto intersection = line_a.intersection(line_b);
		if(!codirection(*a.left() - intersection, *a.right() - intersection) && !codirection(*b.left() - intersection, *b.right() - intersection))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	std::optional<Eigen::Vector2d>& Stripe::Segment::left()
	{
		return left_;
	}

	std::optional<Eigen::Vector2d>& Stripe::Segment::right()
	{
		return right_;
	}

	const std::optional<Eigen::Vector2d>& Stripe::Segment::left() const
	{
		return left_;
	}

	const std::optional<Eigen::Vector2d>& Stripe::Segment::right() const
	{
		return right_;
	}

	bool Stripe::Segment::is_new() const
	{
		return is_new_;
	}

	Stripe::Segment& Stripe::Segment::set_new(bool val)
	{
		is_new_ = val;
		return *this;
	}

	bool Stripe::Segment::lies_on(const Eigen::Vector2d& point, double prec) const
	{
		Line2d line;
		if(right_ && left_)
		{
			line = Line2d::Through(*left_, *right_);
		}
		else
		{
			line = Line2d(Vec2d::UnitX(), left_ ? *left_ : *right_);
		}

		bool on_segment_cond_0 = left_ && right_ && is_approx((point - *left_).norm() + (point - *right_).norm(), vector().norm(), prec);
		bool on_segment_cond_1 = !left_ && right_ && codirection(point - *right_, Vec2d::UnitY(), prec);
		bool on_segment_cond_2 = left_ && !right_ && codirection(point - *left_, Vec2d::UnitY(), prec);

		if(is_approx(line.signedDistance(point), 0, prec) && (on_segment_cond_0 || on_segment_cond_1 || on_segment_cond_2))
		{
			return true;
		}
		else if((right_ && point.isApprox(*right_, prec)) || (left_ && point.isApprox(*left_, prec)))
		{
			return true;
		}
		return false;
	}

	Eigen::Vector2d Stripe::Segment::vector() const
	{
		if(left_ && right_)
		{
			return *right_ - *left_;
		}
		else
		{
			return Vec2d::UnitY();
		}
	}
}
