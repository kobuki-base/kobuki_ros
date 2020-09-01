/**
 * @file /kobuki_node/include/kobuki_node/diagnostics.hpp
 *
 * @brief Diagnostics for the kobuki node.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_node/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_NODE_DIAGNOSTICS_HPP_
#define KOBUKI_NODE_DIAGNOSTICS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>

#include <kobuki_core/packets/cliff.hpp>
#include <kobuki_core/modules/battery.hpp>
#include <kobuki_core/packets/core_sensors.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_node
{

/*****************************************************************************
** Interfaces
*****************************************************************************/


/**
 * Diagnostic checking the robot battery and charging status.
 */
class BatteryTask final : public diagnostic_updater::DiagnosticTask {
public:
  BatteryTask() : DiagnosticTask("Battery") {}
  BatteryTask(BatteryTask && c) = delete;
  BatteryTask & operator=(BatteryTask && c) = delete;
  BatteryTask(const BatteryTask & c) = delete;
  BatteryTask & operator=(const BatteryTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const kobuki::Battery &battery) { status_ = battery; }

private:
  kobuki::Battery status_;
};

/**
 * Simple diagnostic checking to see if kobuki is streaming data or not.
 */
class WatchdogTask final : public diagnostic_updater::DiagnosticTask {
public:
  WatchdogTask() : DiagnosticTask("Watchdog"), alive_(false) {}
  WatchdogTask(WatchdogTask && c) = delete;
  WatchdogTask & operator=(WatchdogTask && c) = delete;
  WatchdogTask(const WatchdogTask & c) = delete;
  WatchdogTask & operator=(const WatchdogTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(bool is_alive) { alive_ = is_alive; }
  bool isAlive() const { return alive_; }

private:
  bool alive_;
};

/**
 * Diagnostic checking the cliff sensors status.
 */
class CliffSensorTask final : public diagnostic_updater::DiagnosticTask {
public:
  CliffSensorTask() : DiagnosticTask("Cliff Sensor"), status_(0) {}
  CliffSensorTask(CliffSensorTask && c) = delete;
  CliffSensorTask & operator=(CliffSensorTask && c) = delete;
  CliffSensorTask(const CliffSensorTask & c) = delete;
  CliffSensorTask & operator=(const CliffSensorTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(uint8_t new_status, const kobuki::Cliff::Data &new_values) {
    status_ = new_status;
    values_ = new_values;
  }

private:
  uint8_t     status_;
  kobuki::Cliff::Data values_;
};

/**
 * Diagnostic checking the wall sensors (aka bumpers) status.
 */
class WallSensorTask final : public diagnostic_updater::DiagnosticTask {
public:
  WallSensorTask() : DiagnosticTask("Wall Sensor"), status_(0) {}
  WallSensorTask(WallSensorTask && c) = delete;
  WallSensorTask & operator=(WallSensorTask && c) = delete;
  WallSensorTask(const WallSensorTask & c) = delete;
  WallSensorTask & operator=(const WallSensorTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(uint8_t new_status) { status_ = new_status; }

private:
  uint8_t status_;
};

/**
 * Diagnostic checking whether the wheels stay in contact with the ground.
 */
class WheelDropTask final : public diagnostic_updater::DiagnosticTask {
public:
  WheelDropTask() : DiagnosticTask("Wheel Drop"), status_(0) {}
  WheelDropTask(WheelDropTask && c) = delete;
  WheelDropTask & operator=(WheelDropTask && c) = delete;
  WheelDropTask(const WheelDropTask & c) = delete;
  WheelDropTask & operator=(const WheelDropTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const uint8_t &new_status) { status_ = new_status; }

private:
  uint8_t status_;
};

/**
 * Diagnostic checking the current supplied to the motors, what
 * can be useful for detecting whether the robot is blocked.
 */
class MotorCurrentTask final : public diagnostic_updater::DiagnosticTask {
public:
  MotorCurrentTask() : DiagnosticTask("Motor Current") {}
  MotorCurrentTask(MotorCurrentTask && c) = delete;
  MotorCurrentTask & operator=(MotorCurrentTask && c) = delete;
  MotorCurrentTask(const MotorCurrentTask & c) = delete;
  MotorCurrentTask & operator=(const MotorCurrentTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const std::vector<uint8_t> &new_values) { values_ = new_values; }

private:
  std::vector<uint8_t> values_;
};

/**
 * Diagnostic checking the on/off state of the motors
 */
class MotorStateTask final : public diagnostic_updater::DiagnosticTask {
public:
  MotorStateTask() : DiagnosticTask("Motor State"), state_(false) {}
  MotorStateTask(MotorStateTask && c) = delete;
  MotorStateTask & operator=(MotorStateTask && c) = delete;
  MotorStateTask(const MotorStateTask & c) = delete;
  MotorStateTask & operator=(const MotorStateTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(bool new_state) { state_ = new_state; };

private:
  bool state_;
};

/**
 * Diagnostic checking the gyro sensor status.
 */
class GyroSensorTask final : public diagnostic_updater::DiagnosticTask {
public:
  GyroSensorTask() : DiagnosticTask("Gyro Sensor"), heading_(0) {}
  GyroSensorTask(GyroSensorTask && c) = delete;
  GyroSensorTask & operator=(GyroSensorTask && c) = delete;
  GyroSensorTask(const GyroSensorTask & c) = delete;
  GyroSensorTask & operator=(const GyroSensorTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(int16_t new_heading) { heading_ = new_heading; }

private:
  int16_t heading_;
};

/**
 * Diagnostic checking the state of the digital input port (four bits).
 */
class DigitalInputTask final : public diagnostic_updater::DiagnosticTask {
public:
  DigitalInputTask() : DiagnosticTask("Digital Input"), status_(0) {}
  DigitalInputTask(DigitalInputTask && c) = delete;
  DigitalInputTask & operator=(DigitalInputTask && c) = delete;
  DigitalInputTask(const DigitalInputTask & c) = delete;
  DigitalInputTask & operator=(const DigitalInputTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(uint16_t new_status) { status_ = new_status; }

private:
  uint16_t status_;
};

/**
 * Diagnostic checking the state of the analog input port (four short integers).
 */
class AnalogInputTask final : public diagnostic_updater::DiagnosticTask {
public:
  AnalogInputTask() : DiagnosticTask("Analog Input") {}
  AnalogInputTask(AnalogInputTask && c) = delete;
  AnalogInputTask & operator=(AnalogInputTask && c) = delete;
  AnalogInputTask(const AnalogInputTask & c) = delete;
  AnalogInputTask & operator=(const AnalogInputTask & c) = delete;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void update(const std::vector<uint16_t> &new_values) { values_ = new_values; }

private:
  std::vector<uint16_t> values_;
};

} // namespace kobuki_node

#endif /* KOBUKI_NODE_DIAGNOSTICS_HPP_ */
