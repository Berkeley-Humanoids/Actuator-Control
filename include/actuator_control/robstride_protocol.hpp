#pragma once

#include <cstdint>

namespace actuator_control {

namespace robstride_comm {
constexpr int GET_DEVICE_ID = 0;
constexpr int OPERATION_CONTROL = 1;
constexpr int OPERATION_STATUS = 2;
constexpr int ENABLE = 3;
constexpr int DISABLE = 4;
constexpr int SET_ZERO_POSITION = 6;
constexpr int SET_DEVICE_ID = 7;
constexpr int READ_PARAMETER = 17;
constexpr int WRITE_PARAMETER = 18;
constexpr int FAULT_REPORT = 21;
constexpr int SAVE_PARAMETERS = 22;
constexpr int SET_BAUDRATE = 23;
constexpr int ACTIVE_REPORT = 24;
constexpr int SET_PROTOCOL = 25;
} // namespace robstride_comm

enum class ParamDtype : uint8_t {
    UINT8,
    INT8,
    UINT16,
    INT16,
    UINT32,
    INT32,
    FLOAT32,
};

struct RobstrideParameter {
    uint16_t param_id;
    ParamDtype dtype;
    const char* name;
};

namespace robstride_param {
constexpr RobstrideParameter MECHANICAL_OFFSET = {0x2005, ParamDtype::FLOAT32, "mechOffset"};
constexpr RobstrideParameter MEASURED_POSITION = {0x3016, ParamDtype::FLOAT32, "mechPos"};
constexpr RobstrideParameter MEASURED_VELOCITY = {0x3017, ParamDtype::FLOAT32, "mechVel"};
constexpr RobstrideParameter MEASURED_TORQUE = {0x302C, ParamDtype::FLOAT32, "torque_fdb"};
constexpr RobstrideParameter MODE = {0x7005, ParamDtype::INT8, "run_mode"};
constexpr RobstrideParameter IQ_TARGET = {0x7006, ParamDtype::FLOAT32, "iq_ref"};
constexpr RobstrideParameter VELOCITY_TARGET = {0x700A, ParamDtype::FLOAT32, "spd_ref"};
constexpr RobstrideParameter TORQUE_LIMIT = {0x700B, ParamDtype::FLOAT32, "limit_torque"};
constexpr RobstrideParameter CURRENT_KP = {0x7010, ParamDtype::FLOAT32, "cur_kp"};
constexpr RobstrideParameter CURRENT_KI = {0x7011, ParamDtype::FLOAT32, "cur_ki"};
constexpr RobstrideParameter CURRENT_FILTER_GAIN = {0x7014, ParamDtype::FLOAT32, "cur_filter_gain"};
constexpr RobstrideParameter POSITION_TARGET = {0x7016, ParamDtype::FLOAT32, "loc_ref"};
constexpr RobstrideParameter VELOCITY_LIMIT = {0x7017, ParamDtype::FLOAT32, "limit_spd"};
constexpr RobstrideParameter CURRENT_LIMIT = {0x7018, ParamDtype::FLOAT32, "limit_cur"};
constexpr RobstrideParameter MECHANICAL_POSITION = {0x7019, ParamDtype::FLOAT32, "mechPos"};
constexpr RobstrideParameter IQ_FILTERED = {0x701A, ParamDtype::FLOAT32, "iqf"};
constexpr RobstrideParameter MECHANICAL_VELOCITY = {0x701B, ParamDtype::FLOAT32, "mechVel"};
constexpr RobstrideParameter VBUS = {0x701C, ParamDtype::FLOAT32, "VBUS"};
constexpr RobstrideParameter POSITION_KP = {0x701E, ParamDtype::FLOAT32, "loc_kp"};
constexpr RobstrideParameter VELOCITY_KP = {0x701F, ParamDtype::FLOAT32, "spd_kp"};
constexpr RobstrideParameter VELOCITY_KI = {0x7020, ParamDtype::FLOAT32, "spd_ki"};
constexpr RobstrideParameter VELOCITY_FILTER_GAIN = {0x7021, ParamDtype::FLOAT32, "spd_filter_gain"};
constexpr RobstrideParameter VEL_ACCELERATION_TARGET = {0x7022, ParamDtype::FLOAT32, "acc_rad"};
constexpr RobstrideParameter PP_VELOCITY_MAX = {0x7024, ParamDtype::FLOAT32, "vel_max"};
constexpr RobstrideParameter PP_ACCELERATION_TARGET = {0x7025, ParamDtype::FLOAT32, "acc_set"};
constexpr RobstrideParameter EPSCAN_TIME = {0x7026, ParamDtype::UINT16, "EPScan_time"};
constexpr RobstrideParameter CAN_TIMEOUT = {0x7028, ParamDtype::UINT32, "canTimeout"};
constexpr RobstrideParameter ZERO_STATE = {0x7029, ParamDtype::UINT8, "zero_sta"};
} // namespace robstride_param

} // namespace actuator_control
