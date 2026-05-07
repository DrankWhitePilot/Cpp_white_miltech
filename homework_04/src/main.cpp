#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    std::ifstream fin(argv[1]);

    if (!fin) {
        std::cerr << "error: cannot open input file\n";
        return 1;
    }

    const double pi = 3.14159265358979323846;
    const double ticks_per_revolution = 1024.0;
    const double wheel_radius_m = 0.3;
    const double wheelbase_m = 1.0;

    const double distance_per_tick =
        2.0 * pi * wheel_radius_m / ticks_per_revolution;

    long timestamp_ms;
    long fl_ticks;
    long fr_ticks;
    long bl_ticks;
    long br_ticks;

    long prev_fl_ticks;
    long prev_fr_ticks;
    long prev_bl_ticks;
    long prev_br_ticks;

    bool is_first_row = true;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    std::cout << std::fixed << std::setprecision(6);

    while (fin >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks) {
        if (is_first_row) {
            prev_fl_ticks = fl_ticks;
            prev_fr_ticks = fr_ticks;
            prev_bl_ticks = bl_ticks;
            prev_br_ticks = br_ticks;

            is_first_row = false;
            continue;
        }

        const long d_fl = fl_ticks - prev_fl_ticks;
        const long d_fr = fr_ticks - prev_fr_ticks;
        const long d_bl = bl_ticks - prev_bl_ticks;
        const long d_br = br_ticks - prev_br_ticks;

        const double d_left_ticks = (d_fl + d_bl) / 2.0;
        const double d_right_ticks = (d_fr + d_br) / 2.0;

        const double dL = d_left_ticks * distance_per_tick;
        const double dR = d_right_ticks * distance_per_tick;

        const double d = (dL + dR) / 2.0;
        const double dtheta = (dR - dL) / wheelbase_m;

        x += d * std::cos(theta + dtheta / 2.0);
        y += d * std::sin(theta + dtheta / 2.0);
        theta += dtheta;

        std::cout << timestamp_ms << " "
                  << x << " "
                  << y << " "
                  << theta << "\n";

        prev_fl_ticks = fl_ticks;
        prev_fr_ticks = fr_ticks;
        prev_bl_ticks = bl_ticks;
        prev_br_ticks = br_ticks;
    }

    if (is_first_row) {
        std::cerr << "error: input file is empty\n";
        return 1;
    }

    return 0;
}