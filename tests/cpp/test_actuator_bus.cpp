#include "actuator_control/actuator_bus.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace actuator_control;

namespace {

class DummyBus : public ActuatorBus {
public:
    using ActuatorBus::ActuatorBus;

    void enable(const std::string& /*motor*/) override {}
    void disable(const std::string& /*motor*/) override {}

    std::tuple<double, double, double> public_process_control(
        const std::string& motor,
        double position,
        double velocity,
        double torque
    ) const {
        return process_mit_control(motor, position, velocity, torque);
    }

    std::pair<double, double> public_process_state(
        const std::string& motor,
        double position,
        double velocity
    ) const {
        return process_mit_state(motor, position, velocity);
    }
};

} // namespace

int main() {
    MotorMap motors = {{"joint", Motor{7, "rs-02"}}};
    Calibration calibration = {{"joint", CalibrationEntry{-1, 0.25}}};
    DummyBus bus("can0", motors, calibration);

    assert(bus.size() == 1);
    assert(bus.models().size() == 1);
    assert(bus.models().at(0) == "rs-02");
    assert(bus.ids().size() == 1);
    assert(bus.ids().at(0) == 7);

    auto [position, velocity, torque] = bus.public_process_control("joint", 1.0, 2.0, 3.0);
    assert(std::fabs(position - (-0.75)) < 1e-9);
    assert(std::fabs(velocity - (-2.0)) < 1e-9);
    assert(std::fabs(torque - (-3.0)) < 1e-9);

    auto [state_position, state_velocity] = bus.public_process_state("joint", -0.75, -2.0);
    assert(std::fabs(state_position - 1.0) < 1e-9);
    assert(std::fabs(state_velocity - 2.0) < 1e-9);

    auto repr = bus.repr();
    assert(repr.find("channel='can0'") != std::string::npos);
    assert(repr.find("Motor{id=7, model='rs-02'}") != std::string::npos);

    bool duplicate_ids_rejected = false;
    try {
        MotorMap duplicates = {
            {"joint_a", Motor{1, "rs-00"}},
            {"joint_b", Motor{1, "rs-01"}},
        };
        DummyBus duplicate_bus("can0", duplicates);
        (void)duplicate_bus;
    } catch (const std::invalid_argument&) {
        duplicate_ids_rejected = true;
    }
    assert(duplicate_ids_rejected);

    return 0;
}
