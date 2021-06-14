#pragma once
#include <vector>
#include <packing/triangle.h>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <optional>
#include <utility>

namespace packing::utility
{
	class StripVisualiser
	{
	public:
		explicit StripVisualiser(double stripe_to_pixel_scale)
			: stripe_to_pixel_scale_(stripe_to_pixel_scale)
			, current_width_(0)
			, current_height_(0)
		{
		}

		~StripVisualiser()
		{
		}

		void display_strip(
			const std::string& filepath,
			double width,
			double height,
			const std::vector<packing::Triangle>& packing,
			const std::optional<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>& fitting_chain
		);

	private:
		void draw_triangle_(cv::Mat img, const Triangle&);
		void display_fitting_chain_(
			cv::Mat img,
			const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& chain,
			const cv::Scalar& color = cv::Scalar{ 0, 205, 0 });

		std::vector<cv::Point> to_cv_(const Triangle&) const;
		std::vector<cv::Point> to_cv_(const std::vector<Eigen::Vector2d>&) const;
		cv::Point to_cv_(const Eigen::Vector2d&) const;
	private:
		double stripe_to_pixel_scale_;
		int current_width_;
		int current_height_;
	};
}