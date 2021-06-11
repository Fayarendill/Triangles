#pragma once
#include <vector>
#include <packing/triangle.h>
#include <opencv2/core.hpp>
#include <Eigen/Dense>

namespace packing::utility
{
	class StripVisualiser
	{
	public:
		explicit StripVisualiser(double stripe_to_pixel_scale)
			:
			stripe_to_pixel_scale_(stripe_to_pixel_scale)
		{
		}

		~StripVisualiser()
		{
		}

		void display_strip(
			const std::string& filepath,
			double width,
			double height,
			const std::vector<packing::Triangle>& packing
		);

	private:
		void draw_triangle_(cv::Mat img, const Triangle&);

		std::vector<cv::Point> to_cv_(const Triangle&) const;
		cv::Point to_cv_(const Eigen::Vector2d&) const;
	private:
		double stripe_to_pixel_scale_;
	};
}