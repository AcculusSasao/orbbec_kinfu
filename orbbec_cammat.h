// MIT License : Copyright (c) 2024 Yukiyoshi Sasao
#pragma once
#include "libobsensor/ObSensor.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>
#include "orbbec_utils.h"

// ref: https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
class OrbbecCameraMatrix
{
public:
	OrbbecCameraMatrix(const OBCameraParam& _ob_param, bool b_coarse=false, bool _b_undistort=false){
		ob_param = _ob_param;
		b_undistort = _b_undistort;
		cfg_m1type = CV_32FC1;	// or CV_16SC2
		cfg_interpolation = cv::INTER_LINEAR;
		cfg_b_coarse = b_coarse;
		prepare();
	}
	void prepare(){
		// depth intrinsic, undistortion parameters
		const OBCameraIntrinsic& di = ob_param.depthIntrinsic;
		const OBCameraDistortion& dd = ob_param.depthDistortion;
		const cv::Size depthFrameSize = cv::Size(di.width, di.height);
		printOBCameraIntrinsic("depth intr: ", di);
		printOBCameraDistortion("depth dist: ", dd);

		// color intrinsic, undistortion parameters
		const OBCameraIntrinsic& ci = ob_param.rgbIntrinsic;
		const OBCameraDistortion& cd = ob_param.rgbDistortion;
		const cv::Size colorFrameSize = cv::Size(ci.width, ci.height);
		printOBCameraIntrinsic("color intr: ", ci);
		printOBCameraDistortion("color dist: ", cd);
		
		depth_intrinsic = (cv::Mat_<double>(3,3) << di.fx, 0, di.cx, 0, di.fy, di.cy, 0, 0, 1);
		color_intrinsic = (cv::Mat_<double>(3,3) << ci.fx, 0, ci.cx, 0, ci.fy, ci.cy, 0, 0, 1);
		if(b_undistort){
			// prepare depth undistortion
			depth_coef = (cv::Mat_<double>(8,1) << dd.k1, dd.k2, dd.p1, dd.p2, dd.k3, dd.k4, dd.k5, dd.k6);
			undistort_depth_intrinsic = depth_intrinsic;
			cv::initUndistortRectifyMap(depth_intrinsic, depth_coef, cv::Matx33d::eye(),
				undistort_depth_intrinsic, depthFrameSize, cfg_m1type, depth_map1, depth_map2);
			
			// prepare color undistortion
			color_coef = (cv::Mat_<double>(8,1) << cd.k1, cd.k2, cd.p1, cd.p2, cd.k3, cd.k4, cd.k5, cd.k6);
			undistort_color_intrinsic = color_intrinsic;
			cv::initUndistortRectifyMap(color_intrinsic, color_coef, cv::Matx33d::eye(),
				undistort_color_intrinsic, colorFrameSize, cfg_m1type, color_map1, color_map2);
		}

		// kinfu params
		kinfu_params = cfg_b_coarse ? cv::kinfu::Params::coarseParams() : cv::kinfu::Params::defaultParams();
		kinfu_params->frameSize = depthFrameSize;
		kinfu_params->intr = b_undistort ? undistort_depth_intrinsic : depth_intrinsic;
		kinfu_params->depthFactor = 1000.f;	// 1000 per 1 meter for Kinect 2 device
		// colored kinfu params
		colored_kinfu_params = cv::colored_kinfu::Params::coloredTSDFParams(cfg_b_coarse);
		colored_kinfu_params->frameSize = depthFrameSize;
		colored_kinfu_params->intr = b_undistort ? undistort_depth_intrinsic : depth_intrinsic;
		colored_kinfu_params->depthFactor = 1000.f;	// 1000 per 1 meter for Kinect 2 device
		colored_kinfu_params->rgb_frameSize = colorFrameSize;
		colored_kinfu_params->rgb_intr = b_undistort ? undistort_color_intrinsic : color_intrinsic;
		
		printf("<kinfu params>\n");
		print_kinfu_params(*kinfu_params.get());
		printf("<colored_kinfu params>\n");
		print_kinfu_params(*colored_kinfu_params.get());
	}
	
	void undistortDepth(cv::Mat& src, cv::Mat& dst){
		cv::remap(src, dst, depth_map1, depth_map2, cfg_interpolation, cv::BORDER_CONSTANT, cv::Scalar());
	}
	void undistortColor(cv::Mat& src, cv::Mat& dst){
		cv::remap(src, dst, color_map1, color_map2, cfg_interpolation, cv::BORDER_CONSTANT, cv::Scalar());
	}
	
	cv::Ptr<cv::kinfu::Params>& getKinfuParams(){
		return kinfu_params;
	}
	cv::Ptr<cv::colored_kinfu::Params>& getColoredKinfuParams(){
		return colored_kinfu_params;
	}
	
	template<typename T>
	static void print_kinfu_params(const T& p){
		printf("frameSize=(%d,%d), volumeType=%d, depthFactor=%f\n", 
			p.frameSize.width, p.frameSize.height, (int)p.volumeType, p.depthFactor);
		printf("bilateral_sigma_depth=%f, bilateral_sigma_spatial=%f, bilateral_kernel_size=%d\n",
			p.bilateral_sigma_depth, p.bilateral_sigma_spatial, p.bilateral_kernel_size);
		printf("pyramidLevels=%d, volumeDims=(%d,%d,%d), voxelSize=%f\n", 
			p.pyramidLevels, p.volumeDims.val[0], p.volumeDims.val[1], p.volumeDims.val[2], p.voxelSize);
		printf("tsdf_min_camera_movement=%f, tsdf_trunc_dist=%f, tsdf_max_weight=%d, raycast_step_factor=%f\n",
			p.tsdf_min_camera_movement, p.tsdf_trunc_dist, p.tsdf_max_weight, p.raycast_step_factor);
		printf("lightPose=(%f,%f,%f)\n",
			p.lightPose.val[0], p.lightPose.val[1], p.lightPose.val[2]);
		printf("icpDistThresh=%f, icpAngleThresh=%f, truncateThreshold=%f\n",
			p.icpDistThresh, p.icpAngleThresh, p.truncateThreshold);
		printf("icpIterations=(");
		for(auto v : p.icpIterations) printf("%d,", v);
		printf(")\n");
	}
	
private:
	OBCameraParam ob_param;
	bool b_undistort;
	int cfg_m1type;
	int cfg_interpolation;
	bool cfg_b_coarse;

	cv::Mat depth_intrinsic, depth_coef;
	cv::Mat undistort_depth_intrinsic;
	cv::Mat depth_map1, depth_map2;

	cv::Mat color_intrinsic, color_coef;
	cv::Mat undistort_color_intrinsic;
	cv::Mat color_map1, color_map2;

	cv::Ptr<cv::kinfu::Params> kinfu_params;
	cv::Ptr<cv::colored_kinfu::Params> colored_kinfu_params;
};
