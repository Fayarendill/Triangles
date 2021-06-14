#include <packing/utility/visualiser.h>

#include <vector>
#include <optional>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <packing/triangle.h>
#include <Eigen/Dense>

namespace packing::utility
{
	void StripVisualiser::display_fitting_chain_(
		cv::Mat img, 
		const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& chain,
		const cv::Scalar& color)
	{
		for(auto& segment : chain)
		{
			cv::line(img, to_cv_(segment.first), to_cv_(segment.second), color, 1, 8);
		}
	}

	void StripVisualiser::display_strip(
		const std::string& filepath,
		double width,
		double height,
		const std::vector<Triangle>& packing,
		const std::optional<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>& fitting_chain
	)
	{
		current_width_ = static_cast<int>(stripe_to_pixel_scale_) * width;
		current_height_= static_cast<int>(stripe_to_pixel_scale_) * height;
		cv::Mat img =
			cv::Mat::ones(current_height_, current_width_, CV_8UC3);

		for(auto& triangle : packing)
		{
			draw_triangle_(std::move(img), triangle);
		}

		if(fitting_chain)
		{
			cv::Mat img = 
				cv::Mat::ones(current_height_, current_width_, CV_8UC3);
			display_fitting_chain_(std::move(img), fitting_chain.value());
			cv::imshow("chain", img);
			cv::waitKey(0);
		}

		cv::imshow("stripe", img);
		cv::waitKey(0);
		cv::destroyAllWindows();
	}

	void StripVisualiser::draw_triangle_(cv::Mat img, const Triangle& triangle)
	{
		cv::polylines(img, to_cv_(triangle), true, cv::Scalar(205, 0, 0), 1, 8);
	}

	std::vector<cv::Point> StripVisualiser::to_cv_(const Triangle& triangle) const
	{
		return std::vector<cv::Point>{ to_cv_(triangle.point_a()), to_cv_(triangle.point_b()), to_cv_(triangle.point_c())};
	}

	std::vector<cv::Point> StripVisualiser::to_cv_(const std::vector<Eigen::Vector2d>& vec) const
	{
		std::vector<cv::Point> cv_points;
		for(auto& point : vec)
		{
			cv_points.push_back(to_cv_(point));
		}
		return cv_points;
	}

	cv::Point StripVisualiser::to_cv_(const Eigen::Vector2d& vec) const
	{
		return cv::Point{ static_cast<int>(vec.x() * stripe_to_pixel_scale_), current_height_ - static_cast<int>(vec.y() * stripe_to_pixel_scale_) };
	}
}