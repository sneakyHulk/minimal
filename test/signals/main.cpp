#include <thread>

#include "common_output.h"

#include "camera/camera.h"
#include "yolo/yolo.h"

class Transform : public InputOutputNode<int, int> {
	int function(int const& in) final { return in * 2; }
};

int main() {
	Camera cam;
	Yolo yol;
	Transform trans;
	cam += yol;
	cam += trans;
	trans += yol;

	std::thread yolo_thread(&Yolo::operator(), &yol);
	std::thread trans_thread(&Transform::operator(), &trans);

	cam();
}