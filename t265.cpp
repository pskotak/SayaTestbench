#include <iostream>
#include "t265.h"

namespace t265 {

std::string t265_serial_number = "925122110508";
std::atomic<bool> ShutdownT265(false);

std::unique_lock<std::mutex> T265_lock(T265_mutex, std::defer_lock);

std::atomic<bool> NewT265(false);
float rs_pitch;
float rs_roll;
float rs_yaw; // Hodnota korigovana magnetometrem
float rs_velocity; // [m/s]
float rs_angular_velo; // [rad/s]
float rs_x;
float rs_y;
float rs_z;
float rs_angVeloX;
float rs_angVeloY;
float rs_angVeloZ;

// Raw data v souradnem systemu T265
float rs_pitch_rad;
float rs_roll_rad;
float rs_yaw_rad;
float rs_raw_x;
float rs_raw_y;
float rs_raw_z;
float rs_raw_aVelo_x;
float rs_raw_aVelo_y;
float rs_raw_aVelo_z;

// glm::vec3 BotPos;
//         BotPos.x = Telemetry.Pose.translation.x;
//         BotPos.y = Telemetry.Pose.translation.y;
//         BotPos.z = Telemetry.Pose.translation.z;
//
//         w = Telemetry.Pose.rotation.w;
//         x = Telemetry.Pose.rotation.x;
//         y = Telemetry.Pose.rotation.y;
//         z = Telemetry.Pose.rotation.z;
//
//         glm::quat BotOrientation;
//         BotOrientation.w = w; BotOrientation.x = x; BotOrientation.y = y; BotOrientation.z = z;
//         BotOrientation = glm::normalize(BotOrientation);

glm::vec3 rs_BotPos;
glm::quat rs_BotOrientation;

// ============================================================================
void RunT265() {
    std::cout << "T265 thread started." << std::endl;

    rs2::pipeline t265_pipe;
    rs2::config t265_cfg;
    t265_cfg.enable_device(t265_serial_number);
    t265_cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF); // T265 pose stream doesn't have resolution/framerate parameters like image streams

    t265_pipe.start(t265_cfg);
    rs2::frameset t265_frames;

    while (!ShutdownT265) {
        t265_frames = t265_pipe.wait_for_frames();
        rs2::pose_frame pose_frame = t265_frames.get_pose_frame();
        if (pose_frame) {
            if (T265_lock.try_lock()) {
// CalcEulerPose
                struct rs2_pose Pose;
                Pose = pose_frame.as<rs2::pose_frame>().get_pose_data();

                auto w = Pose.rotation.w;
                auto x = Pose.rotation.x;
                auto y = Pose.rotation.y;
                auto z = Pose.rotation.z;

                rs_BotOrientation.w = w; rs_BotOrientation.x = x; rs_BotOrientation.y = y; rs_BotOrientation.z = z;
                rs_BotOrientation = glm::normalize(rs_BotOrientation);

                w = Pose.rotation.w;
                x = -Pose.rotation.z;
                y = Pose.rotation.x;
                z = -Pose.rotation.y;

                rs_pitch_rad =  -asin(2.0 * (x*z - w*y));
                rs_roll_rad  =  atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z);
                rs_yaw_rad =  atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z);

                rs_pitch =  rs_pitch_rad * 180.0 / M_PI;
                rs_roll  =  rs_roll_rad * 180.0 / M_PI;
                double yaw =  rs_yaw_rad * 180.0 / M_PI;
                //rs_yaw = YawInit + yaw;
                rs_yaw = yaw;

// Pose
//                 BotPos.x = Pose.translation.x;
//                 BotPos.y = Pose.translation.y;
//                 BotPos.z = Pose.translation.z;

                // Round to centimeters
                BotPos.x = round(Pose.translation.x * 100.0) / 100.0;
                BotPos.y = round(Pose.translation.y) / 100.0;
                BotPos.z = round(Pose.translation.z) / 100.0;

                rs_raw_x = Pose.translation.x; rs_raw_y = Pose.translation.y; rs_raw_z = Pose.translation.z;
                rs2_vector in_v;
                in_v.x = rs_raw_x; in_v.y = rs_raw_y; in_v.z = rs_raw_z;
                rs2_vector rs_v = T265toSys(in_v);
                rs_x = rs_v.x; rs_y = rs_v.y; rs_z = rs_v.z;
// Angular velocity
                rs_raw_aVelo_x = Pose.angular_velocity.x; rs_raw_aVelo_y = Pose.angular_velocity.y; rs_raw_aVelo_z = Pose.angular_velocity.z;
                in_v.x = rs_raw_aVelo_x; in_v.y = rs_raw_aVelo_y; in_v.z = rs_raw_aVelo_z;
                rs_v = T265toSys(in_v);
                rs_angVeloX = rs_v.x; rs_angVeloY = rs_v.y; rs_angVeloZ = rs_v.z;

// Calc Translational Velocity
                rs_velocity =  sqrt(pow(Pose.velocity.x,2.0) + pow(Pose.velocity.y,2.0) + pow(Pose.velocity.z,2.0));

// Calc Angular Velocity
                rs_angular_velo =  sqrt(pow(Pose.angular_velocity.x,2.0) + pow(Pose.angular_velocity.y,2.0) + pow(Pose.angular_velocity.z,2.0));

                T265_lock.unlock();
                NewT265 = true;
            }
        }
    }
    std::cout << "T265 thread ended." << std::endl;
}

} // end namespace
