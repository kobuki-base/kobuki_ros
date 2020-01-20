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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "kobuki_node/diagnostics.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_node
{

/*****************************************************************************
** Implementation
*****************************************************************************/

void BatteryTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  switch ( status.level() ) {
    case ( kobuki::Battery::Maximum ) : {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Maximum");
      break;
    }
    case ( kobuki::Battery::Healthy ) : {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Healthy");
      break;
    }
    case ( kobuki::Battery::Low ) : {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Low");
      break;
    }
    case ( kobuki::Battery::Dangerous ) : {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Dangerous");
      break;
    }
  }

  stat.add("Voltage (V)", status.voltage);
  stat.add("Percent", status.percent());
  stat.add("Charge (Ah)", (2.2*status.percent())/100.0);
  stat.add("Capacity (Ah)", 2.2); // TODO: how can we tell which battery is in use?

  switch (status.charging_source ) {
    case(kobuki::Battery::None) : {
      stat.add("Source", "None");
      break;
    }
    case(kobuki::Battery::Adapter) : {
      stat.add("Source", "Adapter");
      break;
    }
    case(kobuki::Battery::Dock) : {
      stat.add("Source", "Dock");
      break;
    }
  }
  switch ( status.charging_state ) {
    case ( kobuki::Battery::Charged ) : {
      stat.add("Charging State", "Trickle Charging"); // i.e. fully charged
      stat.add("Current (A)", 3.14); // TODO: what's the real value for our charger?
      break;
    }
    case ( kobuki::Battery::Charging ) : {
      stat.add("Charging State", "Full Charging");
      stat.add("Current (A)", 3.14); // TODO: what's the real value for our charger?
      break;
    }
    case ( kobuki::Battery::Discharging ) : {
      stat.add("Charging State", "Not Charging");
      stat.add("Current (A)", 0.0);
      break;
    }
    default: break;
  }
}

void WatchdogTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( alive ) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Alive");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No Signal");
  }
}

void CliffSensorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( status ) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Cliff Detected!");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",   "Reading: %d  Cliff: %s", values.bottom[0], status & kobuki::CoreSensors::Flags::LeftCliff ? "YES" : "NO");
  stat.addf("Center", "Reading: %d  Cliff: %s", values.bottom[1], status & kobuki::CoreSensors::Flags::CenterCliff ? "YES" : "NO");
  stat.addf("Right",  "Reading: %d  Cliff: %s", values.bottom[2], status & kobuki::CoreSensors::Flags::RightCliff ? "YES" : "NO");
}

void WallSensorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( status ) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Wall Hit!");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",   status & kobuki::CoreSensors::Flags::LeftBumper ? "YES" : "NO");
  stat.addf("Center", status & kobuki::CoreSensors::Flags::CenterBumper ? "YES" : "NO");
  stat.addf("Right",  status & kobuki::CoreSensors::Flags::RightBumper ? "YES" : "NO");
}

void WheelDropTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( status ) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Wheel Drop!");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",   status & kobuki::CoreSensors::Flags::LeftWheel ? "YES" : "NO");
  stat.addf("Right",  status & kobuki::CoreSensors::Flags::RightWheel ? "YES" : "NO");
}

void MotorCurrentTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( std::max(values[0], values[1]) > 6 ) { // TODO not sure about this threshold; should be a parameter?
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Is robot stalled? Motors current is very high");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All right");
  }

  stat.addf("Left",  "%d", values[0]);
  stat.addf("Right", "%d", values[1]);
}

void MotorStateTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if ( state == true ) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Motors Enabled");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Motors Disabled");
  }

  stat.addf("State", "%d", int(state));
}

void GyroSensorTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  // Raw data angles are in hundredths of degree
  stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "Heading: %.2f degrees", heading/100.0);
}

void DigitalInputTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "[%d, %d, %d, %d]",
                status & 0x08?1:0, status & 0x04?1:0,
                status & 0x02?1:0, status & 0x01?1:0);
}

void AnalogInputTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "[%d, %d, %d, %d]",
                values[0], values[1], values[2], values[3]);
}

} // namespace kobuki_node
