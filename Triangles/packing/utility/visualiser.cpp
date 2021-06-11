#include <packing/utility/visualiser.h>

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <packing/triangle.h>
#include <Eigen/Dense>

namespace packing::utility
{
	void StripVisualiser::display_strip(
		const std::string& filepath,
		double width,
		double height,
		const std::vector<Triangle>& packing
	)
	{
		cv::Mat img = cv::Mat::zeros(static_cast<int>(stripe_to_pixel_scale_) * width, height * static_cast<int>(stripe_to_pixel_scale_), CV_8UC3);
		for(auto& triangle : packing)
		{
			draw_triangle_(std::move(img), triangle);
		}

		cv::imshow("img", img);
		cv::waitKey(0);
		cv::destroyAllWindows();
	}

	void StripVisualiser::draw_triangle_(cv::Mat img, const Triangle& triangle)
	{
		cv::polylines(img, to_cv_(triangle), true, cv::Scalar(255, 255, 255), 2, 8);
	}

	std::vector<cv::Point> StripVisualiser::to_cv_(const Triangle& triangle) const
	{
		return std::vector<cv::Point>{ to_cv_(triangle.point_a()), to_cv_(triangle.point_b()), to_cv_(triangle.point_c())};
	}

	cv::Point StripVisualiser::to_cv_(const Eigen::Vector2d& vec) const
	{
		return cv::Point{ static_cast<int>(vec.x() * stripe_to_pixel_scale_), static_cast<int>(vec.y() * stripe_to_pixel_scale_) };
	}
}