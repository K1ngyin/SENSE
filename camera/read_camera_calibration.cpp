#include <OpenNI.h>
#include <OniCTypes.h>
#include <OniProperties.h>

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

void writeArray(std::ostream& out, const char* name, const float* values,
                int count) {
    out << name << " = [";
    for (int i = 0; i < count; ++i) {
        if (i != 0) out << ", ";
        out << values[i];
    }
    out << "]\n";
}

void writeDepthToRgbTransform(std::ostream& out, const OBCameraParams& params) {
    // The SDK names its stored transform r2l. With the Astra Pro Plus mapping
    // used by this project (left = depth/IR, right = RGB), invert it to get
    // depth -> RGB: R_l2r = transpose(R_r2l), t_l2r = -R_l2r * t_r2l.
    float rotation[9];
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation[row * 3 + col] = params.r2l_r[col * 3 + row];
        }
    }

    float translation[3] = {0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            translation[row] -= rotation[row * 3 + col] * params.r2l_t[col];
        }
    }

    out << "\n# Derived from r2l; assumes left=Depth/IR and right=RGB.\n";
    out << "# Transform convention: point_rgb = R_depth_to_rgb * point_depth + t_depth_to_rgb\n";
    writeArray(out, "depth_to_rgb_R_row_major", rotation, 9);
    writeArray(out, "depth_to_rgb_t_sdk_units", translation, 3);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string output_path =
        argc > 1 ? argv[1] : "camera/camera_calibration.txt";
    if (argc > 2) {
        std::cerr << "Usage: read_camera_calibration [output.txt]\n"
                  << "Default output: camera/camera_calibration.txt\n";
        return 2;
    }

    openni::Status status = openni::OpenNI::initialize();
    if (status != openni::STATUS_OK) {
        std::cerr << "OpenNI initialization failed: "
                  << openni::OpenNI::getExtendedError() << "\n";
        return 1;
    }

    int result = 0;
    {
        openni::Device device;
        if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
            std::cerr << "Could not open an OpenNI camera: "
                      << openni::OpenNI::getExtendedError() << "\n";
            result = 1;
        } else {
            OBCameraParams params = {};
            int data_size = static_cast<int>(sizeof(params));
            status = device.getProperty(
                openni::OBEXTENSION_ID_CAM_PARAMS,
                reinterpret_cast<std::uint8_t*>(&params), &data_size);
            if (status != openni::STATUS_OK) {
                std::cerr << "Could not read camera calibration: "
                          << openni::OpenNI::getExtendedError() << "\n";
                result = 1;
            } else if (data_size < static_cast<int>(sizeof(params))) {
                std::cerr << "Camera returned an incomplete calibration block ("
                          << data_size << " bytes; expected at least "
                          << sizeof(params) << ").\n";
                result = 1;
            } else {
                std::ofstream out(output_path.c_str(),
                                  std::ios::out | std::ios::trunc);
                if (!out) {
                    std::cerr << "Could not create output file: "
                              << output_path
                              << " (run from the project root or pass an output path)\n";
                    result = 1;
                } else {
                    out << std::fixed << std::setprecision(9);
                    const openni::DeviceInfo& info = device.getDeviceInfo();
                    out << "Orbbec camera calibration\n"
                        << "device_name = " << info.getName() << "\n"
                        << "vendor = " << info.getVendor() << "\n"
                        << "uri = " << info.getUri() << "\n"
                        << "usb_vendor_id = 0x" << std::hex
                        << info.getUsbVendorId() << "\n"
                        << "usb_product_id = 0x" << info.getUsbProductId()
                        << std::dec << "\n"
                        << "property_data_size_bytes = " << data_size << "\n"
                        << "\n# Intrinsic arrays are [fx, fy, cx, cy].\n"
                        << "# Distortion arrays are [k1, k2, p1, p2, k3].\n";
                    writeArray(out, "depth_ir_intrinsics_l", params.l_intr_p, 4);
                    writeArray(out, "rgb_intrinsics_r", params.r_intr_p, 4);
                    writeArray(out, "depth_ir_distortion_l", params.l_k, 5);
                    writeArray(out, "rgb_distortion_r", params.r_k, 5);
                    out << "\n# Raw SDK extrinsic r2l (right camera to left camera).\n"
                        << "# Rotation is row-major; translation units are SDK-defined.\n";
                    writeArray(out, "r2l_R_row_major", params.r2l_r, 9);
                    writeArray(out, "r2l_t_sdk_units", params.r2l_t, 3);
                    writeDepthToRgbTransform(out, params);
                    if (!out) {
                        std::cerr << "Error writing calibration file: "
                                  << output_path << "\n";
                        result = 1;
                    } else {
                        std::cout << "Calibration saved to " << output_path
                                  << "\n";
                    }
                }
            }
            device.close();
        }
    }
    openni::OpenNI::shutdown();
    return result;
}
