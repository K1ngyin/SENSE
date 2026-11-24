#include <OpenNI.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#ifndef OPENNI2_REDIST_DEFAULT
#define OPENNI2_REDIST_DEFAULT ""
#endif

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: openni_depth_capture <output.raw> <timeout_seconds>\n";
        return 2;
    }

    const std::string output_path(argv[1]);
    const int timeout_ms = static_cast<int>(std::stod(argv[2]) * 1000.0);
    if (timeout_ms <= 0) {
        std::cerr << "Timeout must be greater than zero.\n";
        return 2;
    }

    if (OPENNI2_REDIST_DEFAULT[0] != '\0') {
        setenv("OPENNI2_REDIST", OPENNI2_REDIST_DEFAULT, 0);
    }
    openni::Status status = openni::OpenNI::initialize();
    if (status != openni::STATUS_OK) {
        std::cerr << "OpenNI initialize failed: "
                  << openni::OpenNI::getExtendedError() << "\n";
        return 1;
    }

    int exit_code = 0;
    {
        openni::Device device;
        openni::VideoStream depth_stream;
        if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
            std::cerr << "Could not open an OpenNI device: "
                      << openni::OpenNI::getExtendedError() << "\n";
            exit_code = 1;
        } else if (!device.getSensorInfo(openni::SENSOR_DEPTH)) {
            std::cerr << "The OpenNI device does not expose a depth sensor.\n";
            exit_code = 1;
        } else {
            bool depth_to_color_enabled = false;
            if (device.isImageRegistrationModeSupported(
                    openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
                status = device.setImageRegistrationMode(
                    openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
                depth_to_color_enabled =
                    status == openni::STATUS_OK &&
                    device.getImageRegistrationMode() ==
                        openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
            }
            if (!depth_to_color_enabled) {
                std::cerr << "Warning: this device/firmware does not expose "
                             "depth-to-color registration; returning native depth coordinates.\n";
            }

            if (depth_stream.create(device, openni::SENSOR_DEPTH) !=
                openni::STATUS_OK) {
                std::cerr << "Could not create depth stream: "
                          << openni::OpenNI::getExtendedError() << "\n";
                exit_code = 1;
            } else if (depth_stream.setMirroringEnabled(false) !=
                       openni::STATUS_OK) {
                std::cerr << "Could not disable depth stream mirroring: "
                          << openni::OpenNI::getExtendedError() << "\n";
                exit_code = 1;
            } else if (depth_stream.start() != openni::STATUS_OK) {
                std::cerr << "Could not start depth stream: "
                          << openni::OpenNI::getExtendedError() << "\n";
                exit_code = 1;
            } else {
                openni::VideoStream* streams[] = {&depth_stream};
                int ready_index = -1;
                status = openni::OpenNI::waitForAnyStream(
                    streams, 1, &ready_index, timeout_ms);
                if (status != openni::STATUS_OK) {
                    std::cerr << "Timed out waiting for depth frame: "
                              << openni::OpenNI::getExtendedError() << "\n";
                    exit_code = 1;
                } else {
                    openni::VideoFrameRef frame;
                    status = depth_stream.readFrame(&frame);
                    if (status != openni::STATUS_OK || !frame.isValid()) {
                        std::cerr << "Could not read depth frame: "
                                  << openni::OpenNI::getExtendedError() << "\n";
                        exit_code = 1;
                    } else {
                        const int width = frame.getWidth();
                        const int height = frame.getHeight();
                        const int stride = frame.getStrideInBytes();
                        if (width <= 0 || height <= 0 ||
                            stride < width * static_cast<int>(sizeof(std::uint16_t))) {
                            std::cerr << "Invalid depth frame dimensions or stride.\n";
                            exit_code = 1;
                        } else {
                            std::ofstream output(output_path.c_str(),
                                                 std::ios::binary | std::ios::trunc);
                            if (!output) {
                                std::cerr << "Could not create output file: "
                                          << output_path << "\n";
                                exit_code = 1;
                            } else {
                                const auto* data = static_cast<const std::uint8_t*>(
                                    frame.getData());
                                for (int row = 0; row < height; ++row) {
                                    output.write(
                                        reinterpret_cast<const char*>(
                                            data + row * stride),
                                        width * static_cast<int>(sizeof(std::uint16_t)));
                                }
                                if (!output) {
                                    std::cerr << "Error writing depth frame.\n";
                                    exit_code = 1;
                                } else {
                                    const openni::PixelFormat pixel_format =
                                        frame.getVideoMode().getPixelFormat();
                                    const char* units = "RAW";
                                    if (pixel_format == openni::PIXEL_FORMAT_DEPTH_1_MM) {
                                        units = "MM";
                                    } else if (pixel_format ==
                                               openni::PIXEL_FORMAT_DEPTH_100_UM) {
                                        units = "100UM";
                                    }
                                    std::cout << "FRAME " << width << " " << height
                                              << " " << units << " "
                                              << (depth_to_color_enabled ? 1 : 0)
                                              << "\n";
                                }
                            }
                        }
                    }
                }
            }
        }

        if (depth_stream.isValid()) {
            depth_stream.stop();
            depth_stream.destroy();
        }
        if (device.isValid()) {
            device.close();
        }
    }
    openni::OpenNI::shutdown();
    return exit_code;
}
