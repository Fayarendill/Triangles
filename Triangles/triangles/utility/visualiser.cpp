#include <triangles/utility/visualiser.h>

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

namespace packing::utility
{
	void StripVisualiser::display_strip(
		const std::string& filepath,
		double width,
		double height,
		const std::vector<triangles::Triangle>& packing
	)
	{
		cv::Mat img = cv::Mat::zeros(stripe_to_pixel_scale_ * width, height * stripe_to_pixel_scale_, CV_8UC3);
		for(auto& triangle : packing)
		{
			draw_triangle_(std::move(img), triangle);
		}

		cv::imshow("img", img);
		cv::waitKey(0);
		cv::destroyAllWindows();
	}

	void StripVisualiser::draw_triangle_(cv::Mat img, const triangles::Triangle& triangle)
	{
		cv::polylines(img, to_cv_(triangle), true, cv::Scalar(255, 255, 255), 2, 8);
	}

	std::vector<cv::Point> StripVisualiser::to_cv_(const triangles::Triangle& triangle) const
	{
		return std::vector<cv::Point>{ to_cv_(triangle.point_a()), to_cv_(triangle.point_b()), to_cv_(triangle.point_c())};
	}

	cv::Point StripVisualiser::to_cv_(const Eigen::Vector2d& vec) const
	{
		return cv::Point{ vec.x() * stripe_to_pixel_scale_, vec.y() * stripe_to_pixel_scale_ };
	}
}