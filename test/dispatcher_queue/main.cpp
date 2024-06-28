#include <thread>

#include "camera/camera.h"
#include "common_output.h"
#include "tracking/tracking.h"
#include "transformation/Config.h"
#include "transformation/transformation.h"
#include "transformation/undistort.h"
#include "visualization/tracking_visualization.h"
#include "visualization/visualization.h"
#include "yolo/yolo.h"

int main() {
	Config config = make_config();

	CameraSimulator cam_n("s110_n_cam_8");
	CameraSimulator cam_o("s110_o_cam_8");
	CameraSimulator cam_s("s110_s_cam_8");
	CameraSimulator cam_w("s110_w_cam_8");
	Yolo yolo;
	UndistortDetections undistort(config);
	SortTracking track(config);
	ImageTrackingTransformation trans(config);
	Visualization2D vis(config);
	ImageTrackingVisualizationHelper track_vis_helper;
	ImageTrackingVisualization track_vis(track_vis_helper, "s110_s_cam_8");

	cam_n += yolo;
	cam_o += yolo;
	cam_s += yolo;
	cam_w += yolo;

	yolo += undistort;
	undistort += track;

	// yolo += track;
	// cam_s += track_vis_helper;
	// track += track_vis;

	track += trans;

	trans += vis;

	std::thread cam_n_thread(&CameraSimulator::operator(), &cam_n);
	std::thread cam_o_thread(&CameraSimulator::operator(), &cam_o);
	std::thread cam_s_thread(&CameraSimulator::operator(), &cam_s);
	std::thread cam_w_thread(&CameraSimulator::operator(), &cam_w);
	std::thread yolo_thread(&Yolo::operator(), &yolo);
	std::thread undistort_thread(&UndistortDetections::operator(), &undistort);
	std::thread track_thread(&SortTracking::operator(), &track);
	std::thread trans_thread(&ImageTrackingTransformation::operator(), &trans);
	std::thread track_vis_helper_thread(&ImageTrackingVisualizationHelper::operator(), &track_vis_helper);
	std::thread track_vis_thread(&ImageTrackingVisualization::operator(), &track_vis);
	vis();
}