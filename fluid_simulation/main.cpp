#include "sph_demo.h"

int main()
{
    double target_spacing = 0.06;
    int number_of_frames = 200;
    double fps = 60.0;

    auto demo = std::make_shared<Sph_Demo>(target_spacing, number_of_frames, fps);
    demo->run();
	return 0;
}