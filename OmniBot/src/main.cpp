#include "Omni.h"
#include <riplib/riplib.h>

using namespace rip;

double deg_to_rad(double degrees) {
    return degrees * M_PI / 180;
}

int main() {
    RIP::initialize("../config.json");

    auto &omni = Omni::get();

    // omni.turn(90, 0.9, false, false);

    omni.drive_straight(10, 0);
    omni.turn(90);

    omni.drive_with_turn(30, 90, 90);



    RIP::shutdown();
}
