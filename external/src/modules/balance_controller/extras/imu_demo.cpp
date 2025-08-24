#include "../ism330_iio_reader.h"
#include <thread>
#include <chrono>

int main() {
    Ism330IioReader::Config cfg;
    std::printf("Starting IMU.\n");
    cfg.on_sample = [](double pitch, double dpitch, double dyaw, auto ts){
        std::printf("pitch=%.2f° dpitch=%.2f°/s dyaw=%.2f°/s\n",
        pitch*180.0/M_PI, dpitch*180.0/M_PI, dyaw*180.0/M_PI);
    };
    Ism330IioReader imu(cfg);
    std::this_thread::sleep_for(std::chrono::seconds(15));
    return 0;
}
