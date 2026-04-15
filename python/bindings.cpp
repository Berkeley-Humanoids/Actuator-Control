#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "actuator_control/actuator_bus.hpp"
#include "actuator_control/erob_bus.hpp"
#include "actuator_control/motor.hpp"
#include "actuator_control/robstride_bus.hpp"
#include "actuator_control/robstride_protocol.hpp"
#include "actuator_control/sito_bus.hpp"

namespace py = pybind11;
using namespace actuator_control;

namespace {

py::dict calibration_entry_to_python(const CalibrationEntry& entry) {
    py::dict result;
    result["direction"] = entry.direction;
    result["homing_offset"] = entry.homing_offset;
    return result;
}

CalibrationEntry calibration_entry_from_python(const py::handle& value) {
    if (py::isinstance<CalibrationEntry>(value)) {
        return value.cast<CalibrationEntry>();
    }

    if (py::isinstance<py::dict>(value)) {
        auto entry_dict = py::reinterpret_borrow<py::dict>(value);
        if (!entry_dict.contains("direction") || !entry_dict.contains("homing_offset")) {
            throw py::type_error(
                "Calibration dict entries must include 'direction' and 'homing_offset'."
            );
        }
        return CalibrationEntry{
            entry_dict["direction"].cast<int>(),
            entry_dict["homing_offset"].cast<double>(),
        };
    }

    if (py::hasattr(value, "direction") && py::hasattr(value, "homing_offset")) {
        return CalibrationEntry{
            py::getattr(value, "direction").cast<int>(),
            py::getattr(value, "homing_offset").cast<double>(),
        };
    }

    throw py::type_error(
        "Calibration values must be CalibrationEntry objects or mappings with 'direction' and "
        "'homing_offset'."
    );
}

std::optional<Calibration> calibration_from_python(const py::object& obj) {
    if (obj.is_none()) {
        return std::nullopt;
    }
    if (!py::isinstance<py::dict>(obj)) {
        throw py::type_error("calibration must be None or a dict[str, CalibrationEntry-like].");
    }

    Calibration calibration;
    for (auto item : py::reinterpret_borrow<py::dict>(obj)) {
        calibration[item.first.cast<std::string>()] = calibration_entry_from_python(item.second);
    }
    return calibration;
}

py::object calibration_to_python(const std::optional<Calibration>& calibration) {
    if (!calibration) {
        return py::none();
    }

    py::dict result;
    for (const auto& [name, entry] : *calibration) {
        result[py::str(name)] = calibration_entry_to_python(entry);
    }
    return result;
}

py::bytes bytes_from_vector(const std::vector<uint8_t>& bytes) {
    return py::bytes(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

std::string repr_with_class_name(const std::string& repr, const std::string& class_name) {
    constexpr const char* prefix = "ActuatorBus(";
    if (repr.rfind(prefix, 0) == 0) {
        return class_name + repr.substr(std::strlen("ActuatorBus"));
    }
    return repr;
}

template <typename Fn>
auto with_gil_released(Fn&& fn) {
    py::gil_scoped_release release;
    return fn();
}

} // namespace

PYBIND11_MODULE(_actuator_control, m) {
    m.doc() = "Native C++ actuator control core";

    py::class_<Motor>(m, "Motor")
        .def(py::init<int, std::string>(), py::arg("id"), py::arg("model"))
        .def_readwrite("id", &Motor::id)
        .def_readwrite("model", &Motor::model)
        .def("__repr__", [](const Motor& motor) {
            return "Motor(id=" + std::to_string(motor.id) + ", model='" + motor.model + "')";
        });

    py::class_<CalibrationEntry>(m, "CalibrationEntry")
        .def(py::init<int, double>(), py::arg("direction"), py::arg("homing_offset"))
        .def_readwrite("direction", &CalibrationEntry::direction)
        .def_readwrite("homing_offset", &CalibrationEntry::homing_offset);

    py::class_<MotorStatus>(m, "MotorStatus")
        .def(py::init<>())
        .def_readwrite("position", &MotorStatus::position)
        .def_readwrite("velocity", &MotorStatus::velocity)
        .def_readwrite("torque", &MotorStatus::torque)
        .def_readwrite("temperature", &MotorStatus::temperature);

    py::class_<ActuatorBus>(m, "ActuatorBus")
        .def("connect", &ActuatorBus::connect, py::arg("handshake") = true, py::call_guard<py::gil_scoped_release>())
        .def("disconnect", &ActuatorBus::disconnect, py::arg("disable_torque") = true, py::call_guard<py::gil_scoped_release>())
        .def("enable", &ActuatorBus::enable, py::call_guard<py::gil_scoped_release>())
        .def("disable", &ActuatorBus::disable, py::call_guard<py::gil_scoped_release>())
        .def("write_mit_kp_kd", &ActuatorBus::write_mit_kp_kd, py::call_guard<py::gil_scoped_release>())
        .def(
            "write_mit_control",
            &ActuatorBus::write_mit_control,
            py::arg("motor"),
            py::arg("position"),
            py::arg("velocity") = 0.0,
            py::arg("torque") = 0.0,
            py::call_guard<py::gil_scoped_release>()
        )
        .def("read_mit_state", &ActuatorBus::read_mit_state, py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("channel", &ActuatorBus::channel)
        .def_property_readonly("motors", &ActuatorBus::motors)
        .def_property_readonly("calibration", [](const ActuatorBus& bus) {
            return calibration_to_python(bus.calibration());
        })
        .def_property_readonly("bitrate", &ActuatorBus::bitrate)
        .def_property_readonly("models", &ActuatorBus::models)
        .def_property_readonly("ids", &ActuatorBus::ids)
        .def_property_readonly("is_connected", &ActuatorBus::is_connected)
        .def("__len__", &ActuatorBus::size)
        .def("__repr__", &ActuatorBus::repr);

    py::class_<ERobBus, ActuatorBus>(m, "ERobBus")
        .def(
            py::init([](const std::string& channel, const MotorMap& motors, py::object calibration, int bitrate) {
                return std::make_unique<ERobBus>(
                    channel,
                    motors,
                    calibration_from_python(calibration),
                    bitrate
                );
            }),
            py::arg("channel"),
            py::arg("motors"),
            py::arg("calibration") = py::none(),
            py::arg("bitrate") = 1000000
        )
        .def("receive", [](ERobBus& bus, int device_id, double timeout) -> py::object {
            auto result = with_gil_released([&]() { return bus.receive(device_id, timeout); });
            if (!result) {
                return py::none();
            }
            return bytes_from_vector(*result);
        }, py::arg("device_id"), py::arg("timeout") = 0.1)
        .def("transmit", [](ERobBus& bus, int device_id, py::bytes data) {
            std::string payload = data;
            with_gil_released([&]() {
                bus.transmit(device_id, reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
                return 0;
            });
        }, py::arg("device_id"), py::arg("data") = py::bytes("\x00\x00\x00\x00\x00\x00\x00\x00", 8))
        .def("send_command", [](ERobBus& bus, const std::string& motor, int command) {
            with_gil_released([&]() {
                bus.send_command(motor, command);
                return 0;
            });
        }, py::arg("motor"), py::arg("command"))
        .def("read", [](ERobBus& bus, const std::string& motor, int parameter, py::bytes extra_bytes) -> py::object {
            std::string extra = extra_bytes;
            auto result = with_gil_released([&]() {
                return bus.read(
                    motor,
                    parameter,
                    extra.empty() ? nullptr : reinterpret_cast<const uint8_t*>(extra.data()),
                    extra.size()
                );
            });
            if (!result) {
                return py::none();
            }
            return py::int_(*result);
        }, py::arg("motor"), py::arg("parameter"), py::arg("extra_bytes") = py::bytes())
        .def("write", [](ERobBus& bus, const std::string& motor, int parameter, int value) {
            with_gil_released([&]() {
                bus.write(motor, parameter, value);
                return 0;
            });
        }, py::arg("motor"), py::arg("parameter"), py::arg("value"))
        .def("write_subindex", [](ERobBus& bus, const std::string& motor, int parameter, int subindex, int value) {
            with_gil_released([&]() {
                bus.write_subindex(motor, parameter, subindex, value);
                return 0;
            });
        }, py::arg("motor"), py::arg("parameter"), py::arg("subindex"), py::arg("value"))
        .def("__repr__", [](const ERobBus& bus) {
            return repr_with_class_name(bus.repr(), "ERobBus");
        });

    py::enum_<ParamDtype>(m, "ParamDtype")
        .value("UINT8", ParamDtype::UINT8)
        .value("INT8", ParamDtype::INT8)
        .value("UINT16", ParamDtype::UINT16)
        .value("INT16", ParamDtype::INT16)
        .value("UINT32", ParamDtype::UINT32)
        .value("INT32", ParamDtype::INT32)
        .value("FLOAT32", ParamDtype::FLOAT32);

    py::class_<RobstrideParameter>(m, "RobstrideParameter")
        .def_readonly("param_id", &RobstrideParameter::param_id)
        .def_readonly("dtype", &RobstrideParameter::dtype)
        .def_property_readonly("name", [](const RobstrideParameter& param) {
            return std::string(param.name);
        });

    auto robstride_param_module = m.def_submodule("robstride_param", "Robstride parameter descriptors");
    robstride_param_module.attr("MECHANICAL_OFFSET") = py::cast(robstride_param::MECHANICAL_OFFSET);
    robstride_param_module.attr("MEASURED_POSITION") = py::cast(robstride_param::MEASURED_POSITION);
    robstride_param_module.attr("MEASURED_VELOCITY") = py::cast(robstride_param::MEASURED_VELOCITY);
    robstride_param_module.attr("MEASURED_TORQUE") = py::cast(robstride_param::MEASURED_TORQUE);
    robstride_param_module.attr("MODE") = py::cast(robstride_param::MODE);
    robstride_param_module.attr("IQ_TARGET") = py::cast(robstride_param::IQ_TARGET);
    robstride_param_module.attr("VELOCITY_TARGET") = py::cast(robstride_param::VELOCITY_TARGET);
    robstride_param_module.attr("TORQUE_LIMIT") = py::cast(robstride_param::TORQUE_LIMIT);
    robstride_param_module.attr("CURRENT_KP") = py::cast(robstride_param::CURRENT_KP);
    robstride_param_module.attr("CURRENT_KI") = py::cast(robstride_param::CURRENT_KI);
    robstride_param_module.attr("CURRENT_FILTER_GAIN") = py::cast(robstride_param::CURRENT_FILTER_GAIN);
    robstride_param_module.attr("POSITION_TARGET") = py::cast(robstride_param::POSITION_TARGET);
    robstride_param_module.attr("VELOCITY_LIMIT") = py::cast(robstride_param::VELOCITY_LIMIT);
    robstride_param_module.attr("CURRENT_LIMIT") = py::cast(robstride_param::CURRENT_LIMIT);
    robstride_param_module.attr("MECHANICAL_POSITION") = py::cast(robstride_param::MECHANICAL_POSITION);
    robstride_param_module.attr("IQ_FILTERED") = py::cast(robstride_param::IQ_FILTERED);
    robstride_param_module.attr("MECHANICAL_VELOCITY") = py::cast(robstride_param::MECHANICAL_VELOCITY);
    robstride_param_module.attr("VBUS") = py::cast(robstride_param::VBUS);
    robstride_param_module.attr("POSITION_KP") = py::cast(robstride_param::POSITION_KP);
    robstride_param_module.attr("VELOCITY_KP") = py::cast(robstride_param::VELOCITY_KP);
    robstride_param_module.attr("VELOCITY_KI") = py::cast(robstride_param::VELOCITY_KI);
    robstride_param_module.attr("VELOCITY_FILTER_GAIN") = py::cast(robstride_param::VELOCITY_FILTER_GAIN);
    robstride_param_module.attr("VEL_ACCELERATION_TARGET") = py::cast(robstride_param::VEL_ACCELERATION_TARGET);
    robstride_param_module.attr("PP_VELOCITY_MAX") = py::cast(robstride_param::PP_VELOCITY_MAX);
    robstride_param_module.attr("PP_ACCELERATION_TARGET") = py::cast(robstride_param::PP_ACCELERATION_TARGET);
    robstride_param_module.attr("EPSCAN_TIME") = py::cast(robstride_param::EPSCAN_TIME);
    robstride_param_module.attr("CAN_TIMEOUT") = py::cast(robstride_param::CAN_TIMEOUT);
    robstride_param_module.attr("ZERO_STATE") = py::cast(robstride_param::ZERO_STATE);

    auto robstride_comm_module = m.def_submodule("robstride_comm", "Robstride communication types");
    robstride_comm_module.attr("GET_DEVICE_ID") = robstride_comm::GET_DEVICE_ID;
    robstride_comm_module.attr("OPERATION_CONTROL") = robstride_comm::OPERATION_CONTROL;
    robstride_comm_module.attr("OPERATION_STATUS") = robstride_comm::OPERATION_STATUS;
    robstride_comm_module.attr("ENABLE") = robstride_comm::ENABLE;
    robstride_comm_module.attr("DISABLE") = robstride_comm::DISABLE;
    robstride_comm_module.attr("SET_ZERO_POSITION") = robstride_comm::SET_ZERO_POSITION;
    robstride_comm_module.attr("SET_DEVICE_ID") = robstride_comm::SET_DEVICE_ID;
    robstride_comm_module.attr("READ_PARAMETER") = robstride_comm::READ_PARAMETER;
    robstride_comm_module.attr("WRITE_PARAMETER") = robstride_comm::WRITE_PARAMETER;
    robstride_comm_module.attr("FAULT_REPORT") = robstride_comm::FAULT_REPORT;
    robstride_comm_module.attr("SAVE_PARAMETERS") = robstride_comm::SAVE_PARAMETERS;
    robstride_comm_module.attr("SET_BAUDRATE") = robstride_comm::SET_BAUDRATE;
    robstride_comm_module.attr("ACTIVE_REPORT") = robstride_comm::ACTIVE_REPORT;
    robstride_comm_module.attr("SET_PROTOCOL") = robstride_comm::SET_PROTOCOL;

    py::class_<RobstrideBus, ActuatorBus>(m, "RobstrideBus")
        .def(
            py::init([](const std::string& channel, const MotorMap& motors, py::object calibration, int bitrate) {
                return std::make_unique<RobstrideBus>(
                    channel,
                    motors,
                    calibration_from_python(calibration),
                    bitrate
                );
            }),
            py::arg("channel"),
            py::arg("motors"),
            py::arg("calibration") = py::none(),
            py::arg("bitrate") = 1000000
        )
        .def("clear_fault", [](RobstrideBus& bus, const std::string& motor) {
            with_gil_released([&]() {
                bus.clear_fault(motor);
                return 0;
            });
        }, py::arg("motor"))
        .def("disable_nowait", &RobstrideBus::disable_nowait, py::arg("motor"), py::arg("clear_fault") = false)
        .def("disable_with_clear", [](RobstrideBus& bus, const std::string& motor) {
            with_gil_released([&]() {
                bus.disable_with_clear(motor);
                return 0;
            });
        }, py::arg("motor"))
        .def("start_receiver", &RobstrideBus::start_receiver)
        .def("stop_receiver", &RobstrideBus::stop_receiver)
        .def_property_readonly("is_receiver_running", &RobstrideBus::is_receiver_running)
        .def_property("host_id", &RobstrideBus::host_id, &RobstrideBus::set_host_id)
        .def("transmit", [](RobstrideBus& bus, int communication_type, int extra_data, int device_id, py::bytes data) {
            std::string payload = data;
            with_gil_released([&]() {
                bus.transmit(
                    communication_type,
                    extra_data,
                    device_id,
                    reinterpret_cast<const uint8_t*>(payload.data()),
                    payload.size()
                );
                return 0;
            });
        }, py::arg("communication_type"), py::arg("extra_data"), py::arg("device_id"), py::arg("data") = py::bytes("\x00\x00\x00\x00\x00\x00\x00\x00", 8))
        .def("receive", [](RobstrideBus& bus, py::object timeout) -> py::object {
            double timeout_value = timeout.is_none() ? -1.0 : timeout.cast<double>();
            auto result = with_gil_released([&]() { return bus.receive(timeout_value); });
            if (!result) {
                return py::none();
            }
            auto [communication_type, extra_data, device_id, data] = *result;
            return py::make_tuple(communication_type, extra_data, device_id, bytes_from_vector(data));
        }, py::arg("timeout") = py::none())
        .def("receive_status_frame", [](RobstrideBus& bus, const std::string& motor) {
            auto frame = with_gil_released([&]() { return bus.receive_status_frame(motor); });
            return py::make_tuple(frame.position, frame.velocity, frame.torque, frame.temperature);
        }, py::arg("motor"))
        .def("receive_read_frame", [](RobstrideBus& bus) {
            auto data = with_gil_released([&]() { return bus.receive_read_frame(); });
            return bytes_from_vector(data);
        })
        .def("ping_by_id", [](RobstrideBus& bus, int device_id, py::object timeout) -> py::object {
            double timeout_value = timeout.is_none() ? -1.0 : timeout.cast<double>();
            auto result = with_gil_released([&]() { return bus.ping_by_id(device_id, timeout_value); });
            if (!result) {
                return py::none();
            }
            return py::make_tuple(result->first, bytes_from_vector(result->second));
        }, py::arg("device_id"), py::arg("timeout") = py::none())
        .def("read_id", [](RobstrideBus& bus, const std::string& motor, py::object timeout) -> py::object {
            double timeout_value = timeout.is_none() ? -1.0 : timeout.cast<double>();
            auto result = with_gil_released([&]() { return bus.read_id(motor, timeout_value); });
            if (!result) {
                return py::none();
            }
            return py::make_tuple(result->first, bytes_from_vector(result->second));
        }, py::arg("motor"), py::arg("timeout") = py::none())
        .def("write_id", [](RobstrideBus& bus, const std::string& motor, int new_id) -> py::object {
            auto result = with_gil_released([&]() { return bus.write_id(motor, new_id); });
            if (!result) {
                return py::none();
            }
            return py::make_tuple(result->first, bytes_from_vector(result->second));
        }, py::arg("motor"), py::arg("new_id"))
        .def("read", [](RobstrideBus& bus, const std::string& motor, const RobstrideParameter& param) -> py::object {
            auto value = with_gil_released([&]() { return bus.read(motor, param); });
            if (auto* int_value = std::get_if<int>(&value)) {
                return py::int_(*int_value);
            }
            return py::float_(std::get<float>(value));
        }, py::arg("motor"), py::arg("param"))
        .def("write", [](RobstrideBus& bus, const std::string& motor, const RobstrideParameter& param, py::object value) {
            RobstrideValue native_value;
            if (py::isinstance<py::float_>(value)) {
                native_value = value.cast<float>();
            } else {
                native_value = value.cast<int>();
            }
            with_gil_released([&]() {
                bus.write(motor, param, native_value);
                return 0;
            });
        }, py::arg("motor"), py::arg("param"), py::arg("value"))
        .def_static("scan_channel", [](const std::string& channel, int start_id, int end_id, int bitrate) {
            auto result = with_gil_released([&]() {
                return RobstrideBus::scan_channel(channel, start_id, end_id, bitrate);
            });
            py::dict out;
            for (const auto& [device_id, response] : result) {
                out[py::int_(device_id)] = py::make_tuple(response.first, bytes_from_vector(response.second));
            }
            return out;
        }, py::arg("channel"), py::arg("start_id") = 1, py::arg("end_id") = 255, py::arg("bitrate") = 1000000)
        .def("read_fault_status", [](const RobstrideBus& bus, py::object motor) -> py::object {
            if (motor.is_none()) {
                return py::cast(bus.read_fault_status());
            }
            return py::cast(bus.read_fault_status(motor.cast<std::string>()));
        }, py::arg("motor") = py::none())
        .def("clear_fault_status", [](RobstrideBus& bus, py::object motor) {
            if (motor.is_none()) {
                bus.clear_fault_status();
            } else {
                bus.clear_fault_status(motor.cast<std::string>());
            }
        }, py::arg("motor") = py::none())
        .def("read_motor_fault_status", static_cast<std::vector<std::string> (RobstrideBus::*)(const std::string&) const>(&RobstrideBus::read_fault_status), py::arg("motor"))
        .def("clear_all_fault_status", static_cast<void (RobstrideBus::*)()>(&RobstrideBus::clear_fault_status))
        .def("transport_counters", [](const RobstrideBus& bus) {
            auto counters = bus.transport_counters();
            py::dict result;
            result["tx_frames"] = py::int_(counters.tx_frames);
            result["rx_frames"] = py::int_(counters.rx_frames);
            return result;
        })
        .def("__repr__", [](const RobstrideBus& bus) {
            return repr_with_class_name(bus.repr(), "RobstrideBus");
        });

    py::class_<SitoBus, ActuatorBus>(m, "SitoBus")
        .def(
            py::init([](const std::string& channel, const MotorMap& motors, py::object calibration, int bitrate, double control_frequency) {
                return std::make_unique<SitoBus>(
                    channel,
                    motors,
                    calibration_from_python(calibration),
                    bitrate,
                    control_frequency
                );
            }),
            py::arg("channel"),
            py::arg("motors"),
            py::arg("calibration") = py::none(),
            py::arg("bitrate") = 1000000,
            py::arg("control_frequency") = 50.0
        )
        .def("__repr__", [](const SitoBus& bus) {
            return repr_with_class_name(bus.repr(), "SitoBus");
        });
}
