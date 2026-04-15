#include "actuator_control/robstride_protocol.hpp"
#include "actuator_control/robstride_table.hpp"

#include <cassert>
#include <cmath>
#include <string>

using namespace actuator_control;

int main() {
    assert(std::fabs(mit_position_table().at("rs-02") - 4 * M_PI) < 1e-12);
    assert(mit_velocity_table().at("rs-04") == 15);
    assert(mit_torque_table().at("rs-06") == 60);
    assert(mit_kp_table().at("rs-03") == 5000.0);
    assert(mit_kd_table().at("rs-05") == 5.0);

    assert(robstride_param::MECHANICAL_OFFSET.param_id == 0x2005);
    assert(robstride_param::ZERO_STATE.param_id == 0x7029);
    assert(robstride_param::ZERO_STATE.dtype == ParamDtype::UINT8);
    assert(std::string(robstride_param::MECHANICAL_VELOCITY.name) == "mechVel");

    assert(robstride_comm::ENABLE == 3);
    assert(robstride_comm::WRITE_PARAMETER == 18);
    assert(robstride_comm::FAULT_REPORT == 21);

    return 0;
}
