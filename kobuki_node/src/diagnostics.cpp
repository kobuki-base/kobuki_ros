/**
 * @file /src/library/diagnostics.cpp
 *
 * @brief Robot diagnostics implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_node/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <algorithm>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <kobuki_core/modules/battery.hpp>
#include <kobuki_core/packets/core_sensors.hpp>

#include "kobuki_node/diagnostics.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_node
{

/*****************************************************************************
** Implementation
*****************************************************************************/

void BatteryTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  switch (status_.level()) {
    case kobuki::Battery::Maximum:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Maximum");
      break;
    case kobuki::Battery::Healthy:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Healthy");
      break;
    case kobuki::Battery::Low:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Low");
      break;
    case kobuki::Battery::Dangerous:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Dangerous");
      break;
  }

  stat.add("Voltage (V)", status_.voltage);
  stat.add("Percent", status_.percent());
  stat.add("Charge (Ah)", (2.2*status_.percent())/100.0);
  stat.add("Capacity (Ah)", 2.2); // TODO: how can we tell which battery is in use?

  switch (status_.charging_source) {
    case kobuki::Battery::None:
      stat.add("Source", "None");
      break;

    case kobuki::Battery::Adapter:
      stat.add("Source", "Adapter");
      break;

    case kobuki::Battery::Dock:
      stat.add("Source", "Dock");
      break;
  }
  switch (status_.charging_state) {
    case kobuki::Battery::Charged:
      stat.add("Charging State", "Trickle Charging"); // i.e. fully charged
      stat.add("Current (A)", 3.14); // TODO: what's the real value for our charger?
      break;

    case kobuki::Battery::Charging:
      stat.add("Charging State", "Full Charging");
      stat.add("Current (A)", 3.14); // TODO: what's the real value for our charger?
      break;

    case kobuki::Battery::Discharging:
      stat.add("Charging State", "Not Charging");
      stat.add("Current (A)", 0.0);
      break;

    default:
      break;
  }
}

void WatchdogTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (alive_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Alive");
  }
  else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No Signal");
  }
}

void CliffSensorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (status_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Cliff Detected!");
  }
  else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",   "Reading: %d  Cliff: %s", values_.bottom[0], status_ & kobuki::CoreSensors::Flags::LeftCliff ? "YES" : "NO");
  stat.addf("Center", "Reading: %d  Cliff: %s", values_.bottom[1], status_ & kobuki::CoreSensors::Flags::CenterCliff ? "YES" : "NO");
  stat.addf("Right",  "Reading: %d  Cliff: %s", values_.bottom[2], status_ & kobuki::CoreSensors::Flags::RightCliff ? "YES" : "NO");
}

void WallSensorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (status_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Wall Hit!");
  }
  else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",   status_ & kobuki::CoreSensors::Flags::LeftBumper ? "YES" : "NO");
  stat.addf("Center", status_ & kobuki::CoreSensors::Flags::CenterBumper ? "YES" : "NO");
  stat.addf("Right",  status_ & kobuki::CoreSensors::Flags::RightBumper ? "YES" : "NO");
}

void WheelDropTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (status_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Wheel Drop!");
  }
  else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",   status_ & kobuki::CoreSensors::Flags::LeftWheel ? "YES" : "NO");
  stat.addf("Right",  status_ & kobuki::CoreSensors::Flags::RightWheel ? "YES" : "NO");
}

void MotorCurrentTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (values_.empty()) {
    // It is possible for this to get call before update(), and thus values_
    // may be empty.  In that case, just return.
    return;
  }

  if (std::max(values_[0], values_[1]) > 6) { // TODO not sure about this threshold; should be a parameter?
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Is robot stalled? Motors current is very high");
  }
  else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",  "%d", values_[0]);
  stat.addf("Right", "%d", values_[1]);
}

void MotorStateTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (state_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Motors Enabled");
  }
  else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Motors Disabled");
  }

  stat.addf("State", "%d", static_cast<int>(state_));
}

void GyroSensorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  // Raw data angles are in hundredths of degree
  stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "Heading: %.2f degrees", heading_/100.0);
}

void DigitalInputTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "[%d, %d, %d, %d]",
                status_ & 0x08?1:0, status_ & 0x04?1:0,
                status_ & 0x02?1:0, status_ & 0x01?1:0);
}

void AnalogInputTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (values_.empty()) {
    // It is possible for this to get call before update(), and thus values_
    // may be empty.  In that case, just return.
    return;
  }

  stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "[%d, %d, %d, %d]",
                values_[0], values_[1], values_[2], values_[3]);
}

} // namespace kobuki_node
