// MIT License : Copyright (c) 2024 Yukiyoshi Sasao
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#endif

#include "orbbec_utils.h"
#include "orbbec_cammat.h"

template<typename T>
static void get_and_show_point_clouds(T& kf, cv::viz::Viz3d& window,
	cv::UMat& points, cv::UMat& normals, int kinfu_show_mode)
{
	kf->getCloud(points, normals);
	if(!points.empty() && !normals.empty()){
		cv::viz::WCloud cloudWidget(points, cv::viz::Color::white());
		cv::viz::WCloudNormals cloudNormals(points, normals, /*level*/1, /*scale*/0.05, cv::viz::Color::gray());
		window.showWidget("cloud", cloudWidget);
		if(kinfu_show_mode > 1){
			window.showWidget("normals", cloudNormals);
		}
		cv::Vec3d volSize = kf->getParams().voxelSize * kf->getParams().volumeDims;
		window.showWidget("cube", cv::viz::WCube(cv::Vec3d::all(0), volSize), kf->getParams().volumePose);
		window.setViewerPose(kf->getPose());
		window.spinOnce(1, true);
	}
}

static double gettimemsec()
{
#ifdef _WIN32
	SYSTEMTIME st;
	GetSystemTime(&st);
	return (double)(st.wHour * 3600000.f + st.wMinute * 60000.f + st.wSecond * 1000.f + st.wMilliseconds);
#else
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return ((double)tv.tv_sec) * 1000 + ((double)tv.tv_usec/1000);
#endif
}

struct APP_PARAMS_T {
	OBAlignMode ob_align_mode;
	uint32_t ob_timeout_ms;
	
	int color_width;
	int depth_width;
	int fps;
	
	int min_depth_mm;
	int max_depth_mm;
	
	int kinfu_mode;
	bool b_kinfu_coarse;
	int kinfu_show_mode;
	bool b_opencl_off;
	bool b_kinfu_reset_in_icp_fail;
	
	double show_scale;
	
	APP_PARAMS_T() : 
		ob_align_mode(ALIGN_D2C_SW_MODE),	// 0:Disabled, 1:HW, 2:SW
		ob_timeout_ms(100),
		
		color_width(1920),	// 3840, 2560, 1920, 1280
		depth_width(640),	// 1024, 640, 512, 320
		fps(15),			// 15, 5
		
		min_depth_mm(0),
		max_depth_mm(5000),
		
		kinfu_mode(0),		// 0:Disabled, 1:depth, 2:colored
		b_kinfu_coarse(false),	// false: precise or true: fast
		kinfu_show_mode(0),		// 0: render, 1: +3D_View, 2: +normals
		b_opencl_off(false),
		b_kinfu_reset_in_icp_fail(false),
		
		show_scale(0.5)
		{}
};

void usage_key()
{
	printf("keys:\n");
	printf("  ESC : quit app\n");
	printf("  s : save depth.ply in depth-kinfu, color.ply in colored-kinfu\n");
	printf("  r : reset kinfu\n");
	printf("  f : freeze 3D View / restore\n");
	printf("  \n");
}
void usage(int argc, char *argv[], APP_PARAMS_T& par)
{
	printf("usage: %s [options]\n", argv[0]);
	printf(" -a [align_mode(%d)]  0:Disabled, 1:HW, 2:SW\n", par.ob_align_mode);
	printf(" -k [kinfu_mode(%d)]  0:Disabled, 1:depth, 2:colored\n", par.kinfu_mode);
	printf(" -kc                  coarse in colored kinfu\n");
	printf(" -kr                  reset kinfu if ICP fails.\n");
	printf(" -ks [kinfu_show_mode(%d)]  0: render, 1: +3D_View, 2: +normals\n", par.kinfu_show_mode);
	printf(" -md [max_depth_mm(%d)]  max depth in mm\n", par.max_depth_mm);
	printf(" -cloff               set openCL off\n");
	printf(" -ss [show_scale(%.2f)]  window show scale\n", par.show_scale);
	printf(" \n");
	usage_key();
}

int main(int argc, char *argv[])
try {
	// parse args
	APP_PARAMS_T par;
	for(int i=1; i<argc; i++){
		if(0==strcmp(argv[i], "--help")){
			usage(argc, argv, par);
			exit(0);
		}
		else if(0==strcmp(argv[i], "-a")){
			par.ob_align_mode = (OBAlignMode)atoi(argv[++i]);
		}
		else if(0==strcmp(argv[i], "-k")){
			par.kinfu_mode = atoi(argv[++i]);
		}
		else if(0==strcmp(argv[i], "-kc")){
			par.b_kinfu_coarse = true;
		}
		else if(0==strcmp(argv[i], "-kr")){
			par.b_kinfu_reset_in_icp_fail = true;
		}
		else if(0==strcmp(argv[i], "-ks")){
			par.kinfu_show_mode = atoi(argv[++i]);
		}
		else if(0==strcmp(argv[i], "-md")){
			par.max_depth_mm = atoi(argv[++i]);
		}
		else if(0==strcmp(argv[i], "-cloff")){
			par.b_opencl_off = true;
		}
		else{
			printf("unknown option %s\n", argv[i]);
			exit(-1);
		}
	}
	
	// print info
	print_ob_info();
	ob::Context ctx;
	auto devList = ctx.queryDeviceList();
	if(devList->deviceCount() == 0) {
		std::cerr << "Device not found!" << std::endl;
		return -1;
	}
	for(uint32_t i=0; i<devList->deviceCount(); i++){
		print_ob_device(i, devList->getDevice(i).get());
	}

	if(par.b_opencl_off){
		// Enables OpenCL explicitly (by default can be switched-off)
		cv::setUseOptimized(false);
	}

	// prepare pipeline
	ob::Pipeline pipe;
	std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

	// set color profile
	std::shared_ptr<ob::StreamProfileList> colorProfiles;
	std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
	if(par.kinfu_mode != 1){
		colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
		//colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
		colorProfile = colorProfiles->getVideoStreamProfile(par.color_width, OB_HEIGHT_ANY, OB_FORMAT_MJPG, par.fps);
		config->enableStream(colorProfile);
	}
	
	// set depth profile
	auto depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
	//auto depthProfiles = pipe.getD2CDepthProfileList(colorProfile, par.ob_align_mode);
	std::shared_ptr<ob::VideoStreamProfile> depthProfile = nullptr;
	try {
		depthProfile = depthProfiles->getVideoStreamProfile(par.depth_width, OB_HEIGHT_ANY, OB_FORMAT_Y16, par.fps);
		//depthProfile = depthProfiles->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, colorProfile->fps());
	}
	catch(ob::Error &e) {
		depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
	}
	config->enableStream(depthProfile);

	if(par.kinfu_mode != 1){
		printf("Profile Color %dx%d fps%d, Depth %dx%d fps%d\n",
			colorProfile->width(), colorProfile->height(), colorProfile->fps(),
			depthProfile->width(), depthProfile->height(), depthProfile->fps()
		);
		config->setAlignMode(par.ob_align_mode);
	}
	else{
		printf("Profile Depth %dx%d fps%d\n",
			depthProfile->width(), depthProfile->height(), depthProfile->fps()
		);
	}
	pipe.start(config);
	
	// camera param
	auto cameraParam = pipe.getCameraParam();	// If D2C is enabled, it will return the camera parameters after D2C
	if(cameraParam.depthIntrinsic.width == 0){
		printf("depth width=0 (The camera may not support HW D2C).\n");
		exit(-1);
	}

	// prepare camera matrix
	std::unique_ptr<OrbbecCameraMatrix> cam = std::unique_ptr<OrbbecCameraMatrix>(new OrbbecCameraMatrix(cameraParam, par.b_kinfu_coarse));

	// prepare kinfu
	cv::Ptr<cv::kinfu::KinFu> kf;
	if(par.kinfu_mode == 1){
		kf = cv::kinfu::KinFu::create(cam->getKinfuParams());
	}
	cv::Ptr<cv::colored_kinfu::ColoredKinFu> kfc;
	if(par.kinfu_mode == 2){
		kfc = cv::colored_kinfu::ColoredKinFu::create(cam->getColoredKinfuParams());
		// Enables OpenCL explicitly (by default can be switched-off)
		//cv::setUseOptimized(true);
	}

	// prepare point cloud
	ob::PointCloudFilter pointCloud;
	pointCloud.setCameraParam(cameraParam);

	// prepare Viz3d
	cv::viz::Viz3d window;
	if(par.kinfu_show_mode > 0){
		const std::string vizWindowName = "Point Cloud";
		window = cv::viz::Viz3d(vizWindowName);
		window.setViewerPose(cv::Affine3f::Identity());
	}

	usage_key();
	cv::UMat points, normals;
	bool b_first = true;
	bool b_pause_3dviz = false;
	while(1) {
		double t0 = gettimemsec();
		auto frameSet = pipe.waitForFrames(par.ob_timeout_ms);
		if(frameSet == nullptr) {
			continue;
		}

		std::shared_ptr<ob::ColorFrame> colorFrame;
		if(par.kinfu_mode != 1){
			colorFrame = frameSet->colorFrame();
			if(colorFrame == nullptr){
				fprintf(stderr, "drop frame bgr=%p\n", colorFrame.get());
				continue;
			}
		}
		auto depthFrame = frameSet->depthFrame();
		if(depthFrame == nullptr){
			fprintf(stderr, "drop frame depth=%p\n", depthFrame.get());
			continue;
		}
		
		double t1 = gettimemsec();
		float depthValueScale = depthFrame->getValueScale();
		truncateDepth(depthFrame.get(), par.min_depth_mm / depthValueScale, par.max_depth_mm / depthValueScale, 0, 0);

		if(b_first){
			printf("capture color=%dx%d, depth=%dx%d, depthValueScale = %f\n", 
				colorFrame.get() ? colorFrame->width() : 0, colorFrame.get() ? colorFrame->height() : 0,
				depthFrame->width(), depthFrame->height(), depthValueScale);
		}
		
		// to cv::Mat
		cv::Mat bgr;
		if(par.kinfu_mode != 1){
			bgr = convObFrame2CvMat(colorFrame.get());
		}
		cv::Mat depth = convObFrame2CvMat(depthFrame.get());

		// undistortion
		//cv::Mat undistort_depth, undistort_bgr;
		//cam->undistortDepth(depth, undistort_depth);
		//cam->undistortColor(bgr, undistort_bgr);

		cv::Mat fuse;
		fuseColorDepth(fuse, bgr, depth);
		//fuseColorDepth(fuse, undistort_bgr, undistort_depth);
		
		double t2 = gettimemsec();
		double t22=0, t23=0;
		// process kinfu
		if(!kf.empty()){
			if(!kf->update(depth)){
				t22 = gettimemsec();
				printf("ICP fails.\n");
				if(par.b_kinfu_reset_in_icp_fail){
					kf->reset();
				}
			}
			else{
				t22 = gettimemsec();
				cv::Mat tsdfRender;
				kf->render(tsdfRender);	// a 0-surface of TSDF using Phong shading
				t23 = gettimemsec();
				showColor("kinfu render", tsdfRender, par.show_scale);
				
				if(par.kinfu_show_mode > 0 && (!b_pause_3dviz)){
					get_and_show_point_clouds(kf, window, points, normals, par.kinfu_show_mode);
				}
			}
		}

		// process colored_kinfu
		if(!kfc.empty()){
			if(!kfc->update(depth, bgr)){
				t22 = gettimemsec();
				printf("ICP fails.\n");
				if(par.b_kinfu_reset_in_icp_fail){
					kfc->reset();
				}
			}
			else{
				t22 = gettimemsec();
				cv::Mat tsdfRender;
				kfc->render(tsdfRender);	// a 0-surface of TSDF using Phong shading
				t23 = gettimemsec();
				showColor("colored_kinfu render", tsdfRender, par.show_scale);
				
				if(par.kinfu_show_mode > 0 && (!b_pause_3dviz)){
					get_and_show_point_clouds(kfc, window, points, normals, par.kinfu_show_mode);
				}
			}
		}
		double t3 = gettimemsec();
		if(b_pause_3dviz){
			window.spinOnce(1, true);
		}

		showDepth("Depth", depth, par.show_scale);
		if(par.kinfu_mode != 1){
			showColor("Color", bgr, par.show_scale);
			showColor("Fuse", fuse, par.show_scale);
		}
		//showColor("Undistorted Color", undistort_bgr, par.show_scale);
		//showDepth("Undistorted Depth", undistort_depth, par.show_scale);

		int key = cv::waitKey(1);
		if(key == 27) break;
		if(key == 'r'){
			printf("kinfu reset\n");
			if(!kf.empty()) kf->reset();
			if(!kfc.empty()) kfc->reset();
		}

		if(key == 's'){
			pointCloud.setPositionDataScaled(depthValueScale);
			if(par.kinfu_mode == 1){
				pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
				std::shared_ptr<ob::Frame> frame = pointCloud.process(frameSet);
				savePointsToPly(frame, "depth.ply");
				printf("depth.ply is saved.\n");
			}
			else if(par.kinfu_mode == 2){
				pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
				std::shared_ptr<ob::Frame> frame = pointCloud.process(frameSet);
				saveRGBPointsToPly(frame, "color.ply");
				printf("color.ply is saved.\n");
			}
		}
		
		if(key == 'f'){
			b_pause_3dviz = !b_pause_3dviz;
		}
		
		b_first = false;
		double t4 = gettimemsec();
		printf("[msec] total:%d, cap:%d, pre:%d, kinfu:%d, show:%d\n",
			(int)(t4-t0), (int)(t1-t0), (int)(t2-t1), (int)(t3-t2), (int)(t4-t3));
		if(t22 != 0 && t23 != 0){
			printf("  (kinfu-only:%d, render:%d)\n", (int)(t22-t2), (int)(t23-t22));
		}
	}

	pipe.stop();

	return 0;
}
catch(ob::Error &e) {
	std::cerr << "ob Exception:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
	exit(EXIT_FAILURE);
}
