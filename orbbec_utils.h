// MIT License : Copyright (c) 2024 Yukiyoshi Sasao
#pragma once
#include "libobsensor/ObSensor.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>	// kinfu, colored_kinfu
#include <opencv2/viz.hpp>

static const char* OBSensorTypeStr[] = {
	"UNKNOWN",
	"IR",
	"COLOR",
	"DEPTH",
	"ACCEL",
	"GYRO",
	"IR_LEFT",
	"IR_RIGHT",
	"RAW_PHASE",
};

static void print_ob_info()
{
	printf("Ob major=%d, minor=%d, patch=%d, stage=%s\n", ob::Version::getMajor(), ob::Version::getMinor(), ob::Version::getPatch(), ob::Version::getStageVersion());
}
static void print_ob_device(int dev_idx, ob::Device* dev)
{
	auto info = dev->getDeviceInfo();
	printf("dev[%d] name=%s, pid=%d, vid=%d, uid=%s, firm=%s, serial=%s, con=%s\n",
		dev_idx, info->name(), info->pid(), info->vid(), info->uid(), info->firmwareVersion(), info->serialNumber(), info->connectionType());
	printf("  SensorType=");
	auto sensorList = dev->getSensorList();
	for(uint32_t i=0; i<sensorList->count(); i++){
		auto sensor = sensorList->getSensor(i);
		printf("%s(%d), ", OBSensorTypeStr[sensor->type()], sensor->type());
	}
	printf("\n");
}

static inline cv::Mat convObFrame2CvMat(ob::VideoFrame* frame, bool* b_bgr=NULL)
{
	cv::Mat mat;
	
	int ch = 0;
	int byte = 0;
	int mattype = 0;
	if(b_bgr) *b_bgr = false;
	switch(frame->format()){
	case OB_FORMAT_Y16:
		ch = 1;
		byte = 2;
		mattype = CV_16UC1;
		break;
	case OB_FORMAT_Y8:
		ch = 1;
		byte = 1;
		mattype = CV_8UC1;
		break;
	case OB_FORMAT_RGB:
		ch = 3;
		byte = 1;
		mattype = CV_8UC3;
		break;
	case OB_FORMAT_BGR:
		ch = 3;
		byte = 1;
		mattype = CV_8UC3;
		if(b_bgr) *b_bgr = true;
		break;
	case OB_FORMAT_MJPG:
		ch = 3;
		byte = 1;
		mattype = CV_8UC3;
		if(b_bgr) *b_bgr = true;
		break;
	default:
		fprintf(stderr, "unknown format %d\n", frame->format());
		break;
	}
	
	uint32_t w = frame->width();
	uint32_t h = frame->height();
	uint32_t expectedSize = w * h * ch * byte;
	if(expectedSize){
		uint32_t size = frame->dataSize();
		if(frame->format() == OB_FORMAT_MJPG){
			mat = cv::imdecode(cv::Mat(1, size, CV_8UC1, frame->data()), cv::IMREAD_COLOR);
		}
		else{
			if(expectedSize != size){
				fprintf(stderr, "The expected size(w=%d,h=%d,ch=%d,byte=%d) %d is different from actual %d.\n", w, h, ch, byte, expectedSize, size);
			}
			mat = cv::Mat(h, w, mattype, frame->data());
		}
	}
	return mat;
}

static void showColor(cv::String winname, cv::Mat& mat, double scale=1.f)
{
	cv::Mat _mat;
	if(scale == 1.f) _mat = mat;
	else cv::resize(mat, _mat, cv::Size(), scale, scale);
	cv::imshow(winname, _mat);
}
static void showDepth(cv::String winname, cv::Mat& mat, double scale=1.f, double alpha=1.f/8.f)
{
	cv::Mat _mat;
	mat.convertTo(_mat, CV_8UC1, alpha);
	cv::Mat __mat;
	if(scale == 1.f) __mat = _mat;
	else cv::resize(_mat, __mat, cv::Size(), scale, scale);
	cv::imshow(winname, __mat);
}

static void truncateDepth(ob::DepthFrame* frame, uint16_t min_value, uint16_t max_value, uint16_t min_default, uint16_t max_default)
{
	uint16_t* pdata = (uint16_t*)frame->data();
	uint32_t dataSize = frame->width() * frame->height();
	for(uint32_t i=0; i<dataSize; i++){
		uint16_t val = *pdata;
		if(val < min_value) val = min_default;
		if(val > max_value) val = max_default;
		*pdata++ = val;
	}
}

static void fuseColorDepth(cv::Mat& dst, cv::Mat& bgr, cv::Mat& depth, const int type=0, const int addVal=80)
{
	if(bgr.empty()){
		dst = depth;
		return;
	}
	if(type == 0){
		cv::Mat mask;
		depth.convertTo(mask, CV_8UC1);
		cv::threshold(mask, mask, 0, addVal, cv::THRESH_BINARY);
		cv::Mat empty = cv::Mat::zeros(mask.size(), CV_8UC1);
		cv::Mat mask2;
		std::vector<cv::Mat> input = {empty, mask, empty};
		cv::merge(input, mask2);
		dst = bgr + mask2;
	}
	else{
		cv::Mat green(bgr.size(), bgr.type());
		green.setTo(cv::Scalar(0, addVal, 0));
		cv::Mat mask;
		depth.convertTo(mask, CV_8UC1);
		cv::add(bgr, green, dst, mask);
	}
}

static inline void printOBCameraIntrinsic(const char* msg, const OBCameraIntrinsic& c)
{
	printf("%s%dx%d, fx=%f,fy=%f,cx=%f,cy=%f\n", msg, c.width, c.height, c.fx, c.fy, c.cx, c.cy);
}
static inline void printOBCameraDistortion(const char* msg, const OBCameraDistortion& d)
{
	printf("%sk1=%f,k2=%f,k3=%f,k4=%f,k5=%f,k6=%f, p1=%f,p2=%f\n", msg, d.k1, d.k2, d.k3, d.k4, d.k5, d.k6, d.p1, d.p2);
}

//// from OrbbecSDK/Example/cpp/Sample-PointCloud/PointCloud.cpp
// Save point cloud data to ply
static void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
	int   pointsSize = frame->dataSize() / sizeof(OBPoint);
	FILE *fp         = fopen(fileName.c_str(), "wb+");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "element vertex %d\n", pointsSize);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "end_header\n");

	OBPoint *point = (OBPoint *)frame->data();
	for(int i = 0; i < pointsSize; i++) {
		fprintf(fp, "%.3f %.3f %.3f\n", point->x, point->y, point->z);
		point++;
	}

	fflush(fp);
	fclose(fp);
}

// Save colored point cloud data to ply
static void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
	int   pointsSize = frame->dataSize() / sizeof(OBColorPoint);
	FILE *fp         = fopen(fileName.c_str(), "wb+");
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "element vertex %d\n", pointsSize);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property uchar red\n");
	fprintf(fp, "property uchar green\n");
	fprintf(fp, "property uchar blue\n");
	fprintf(fp, "end_header\n");

	OBColorPoint *point = (OBColorPoint *)frame->data();
	for(int i = 0; i < pointsSize; i++) {
		fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", point->x, point->y, point->z, (int)point->r, (int)point->g, (int)point->b);
		point++;
	}

	fflush(fp);
	fclose(fp);
}
