#ifndef T265_H
#define T265_H

#include "SayaGlobals/globals.h"

namespace t265 {

extern std::string t265_serial_number;

extern float rs_pitch;
extern float rs_roll;
extern float rs_yaw; // Hodnota korigovana magnetometrem
extern float rs_velocity; // [m/s]
extern float rs_angular_velo; // [rad/s]
extern float rs_x;
extern float rs_y;
extern float rs_z;
extern float rs_angVeloX;
extern float rs_angVeloY;
extern float rs_angVeloZ;

extern glm::vec3 rs_BotPos;
extern glm::quat rs_BotOrientation;

// Raw data v souradnem systemu T265
extern float rs_pitch_rad;
extern float rs_roll_rad;
extern float rs_yaw_rad;
extern float rs_raw_x;
extern float rs_raw_y;
extern float rs_raw_z;

extern std::atomic<bool> NewT265;
extern std::atomic<bool> ShutdownT265;
extern void RunT265();

} // end namespace
#endif
