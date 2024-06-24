#include <thread>

#include "camera/camera.h"
#include "common_output.h"
#include "tracking/tracking.h"
#include "transformation/Config.h"
#include "transformation/transformation.h"
#include "visualization/visualization.h"
#include "yolo/yolo.h"

int main() {
	Config config = make_config();

	Camera cam_n("s110_n_cam_8");
	Camera cam_o("s110_o_cam_8");
	Camera cam_s("s110_s_cam_8");
	Camera cam_w("s110_w_cam_8");
	Yolo yolo;
	UndistortDetections undistort(config);
	SortTracking track(config);
	Visualization2D vis(config);

	cam_n += yolo;
	cam_o += yolo;
	cam_s += yolo;
	cam_w += yolo;

	yolo += undistort;

	undistort += track;

	track += vis;

	std::thread cam_n_thread(&Camera::operator(), &cam_n);
	std::thread cam_o_thread(&Camera::operator(), &cam_o);
	std::thread cam_s_thread(&Camera::operator(), &cam_s);
	std::thread cam_w_thread(&Camera::operator(), &cam_w);
	std::thread yolo_thread(&Yolo::operator(), &yolo);
	std::thread undistort_thread(&UndistortDetections::operator(), &undistort);
	std::thread track_thread(&NoTracking::operator(), &track);
	vis();
}