/**
 * @file /kobuki/src/driver/kobuki.cpp
 *
 * @brief Implementation for the kobuki device driver.
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ecl/math.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/time/sleep.hpp>
#include <ecl/converters.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/time/timestamp.hpp>
#include "../../include/kobuki_driver/kobuki.hpp"
#include "../../include/kobuki_driver/packet_handler/payload_headers.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki
{

/*****************************************************************************
** Implementation [PacketFinder]
*****************************************************************************/

bool PacketFinder::checkSum()
{
  unsigned int packet_size(buffer.size());
  unsigned char cs(0);
  for (unsigned int i = 2; i < packet_size; i++)
  {
    cs ^= buffer[i];
  }
  return cs ? false : true;
}


/*****************************************************************************
 ** Implementation [Kobuki]
 *****************************************************************************/

void Kobuki::init(Parameters &parameters) throw (ecl::StandardException)
{

  if (!parameters.validate())
  {
    throw ecl::StandardException(LOC, ecl::ConfigurationError, "Kobuki's parameter settings did not validate.");
  }
  protocol_version = parameters.protocol_version;
  simulation = parameters.simulation;
  std::string sigslots_namespace = parameters.sigslots_namespace;

  if ( !simulation ) {
    serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
    serial.block(4000); // blocks by default, but just to be clear!
    serial.clear();
    ecl::PushAndPop<unsigned char> stx(2, 0);
    ecl::PushAndPop<unsigned char> etx(1);
    stx.push_back(0xaa);
    stx.push_back(0x55);
    packet_finder.configure(sigslots_namespace, stx, etx, 1, 64, 1, true);
    is_connected = true;
  }

  sig_wheel_state.connect(sigslots_namespace + std::string("/joint_state"));
  sig_core_sensors.connect(sigslots_namespace + std::string("/core_sensors"));
  //sig_serial_timeout.connect(sigslots_namespace+std::string("/serial_timeout"));

  sig_ir.connect(sigslots_namespace + std::string("/ir"));
  sig_dock_ir.connect(sigslots_namespace + std::string("/dock_ir"));
  sig_inertia.connect(sigslots_namespace + std::string("/inertia"));
  sig_cliff.connect(sigslots_namespace + std::string("/cliff"));
  sig_current.connect(sigslots_namespace + std::string("/current"));
  sig_magnet.connect(sigslots_namespace + std::string("/magnet"));
  sig_hw.connect(sigslots_namespace + std::string("/hw"));
  sig_fw.connect(sigslots_namespace + std::string("/fw"));
  sig_time.connect(sigslots_namespace + std::string("/time"));
  sig_eeprom.connect(sigslots_namespace + std::string("/eeprom"));
  sig_gp_input.connect(sigslots_namespace + std::string("/gp_input"));

  sig_debug.connect(sigslots_namespace + std::string("/ros_debug"));
  sig_info.connect(sigslots_namespace + std::string("/ros_info"));
  sig_warn.connect(sigslots_namespace + std::string("/ros_warn"));
  sig_error.connect(sigslots_namespace + std::string("/ros_error"));


  /******************************************
   ** Configuration & Connection Test
   *******************************************/

  last_tick_left = 0;
  last_tick_right = 0;
  last_rad_left = 0.0;
  last_rad_right = 0.0;
  last_mm_left = 0.0;
  last_mm_right = 0.0;

  v = 0;
  w = 0;
  radius = 0;
  speed = 0;
  bias = 0.298; //wheelbase, wheel_to_wheel, in [m]
  wheel_radius = 0.042;
  imu_heading_offset = 0;

  kinematics.reset(new ecl::DifferentialDrive::Kinematics(bias, wheel_radius));

  if ( simulation ) {
    kobuki_sim.init(bias, 1000 * tick_to_rad / tick_to_mm); // bias, metres to radians
  }
  is_running = true;
  start();
}

void Kobuki::close()
{
  disable();
  sig_debug.emit("Device: kobuki driver terminated.");
  return;
}

/**
 * @brief Performs a scan looking for incoming data packets.
 *
 * Sits on the device waiting for incoming and then parses it, and signals
 * that an update has occured.
 *
 * Or, if in simulation, just loopsback the motor devices.
 */

void Kobuki::runnable()
{
  unsigned char buf[256];
  bool get_packet;
  stopwatch.restart();

  /*********************
  ** Simulation Params
  **********************/

  while (is_running)
  {
    get_packet = false;

    if ( simulation ) {
      kobuki_sim.update();
      kobuki_sim.sleep();
      sig_wheel_state.emit();
      sig_inertia.emit();
    } else {
      /*********************
      ** Read Incoming
      **********************/
      int n = serial.read(buf, packet_finder.numberOfDataToRead());
      if (n == 0) {
        ROS_ERROR_STREAM("kobuki_node : no serial data in.");
        continue;
      } else {
        ROS_DEBUG_STREAM("kobuki_node : serial_read(" << n << ")");
        // might be useful to send this to a topic if there is subscribers
//        static unsigned char last_char(buf[0]);
//        for( int i(0); i<n; i++ )
//        {
//          printf("%02x ", buf[i] );
//          if( last_char == 0xaa && buf[i] == 0x55 ) printf("\n");
//          last_char = buf[i];
//        }
      }

      if (packet_finder.update(buf, n)) // this clears packet finder's buffer and transfers important bytes into it
      {
        packet_finder.getBuffer(data_buffer);  // get a reference to packet finder's buffer.

  #if 0
        if( verbose )
        {
          printf("Packet: ");
          for( unsigned int i=0; i<data_buffer.size(); i++ )
          {
            printf("%02x ", data_buffer[i] );
            if( i != 0 && ((i%5)==0) ) printf(" ");
          }
        }
  #endif
        // deserialise; first three bytes are not data.
        data_buffer.pop_front();
        data_buffer.pop_front();
        data_buffer.pop_front();

        if (protocol_version == "2.0")
        {
          while (data_buffer.size() > 1/*size of etx*/)
          {
            // std::cout << "header_id: " << (unsigned int)data_buffer[0] << " | ";
            // std::cout << "remains: " << data_buffer.size() << " | ";
            switch (data_buffer[0])
            {
              // these come with the streamed feedback
              case Header::CoreSensors:
                kobuki_default.deserialise(data_buffer);
                sig_core_sensors.emit();
                sig_wheel_state.emit();
                break;
              case Header::DockInfraRed:
                kobuki_dock_ir.deserialise(data_buffer);
                sig_dock_ir.emit();
                break;
              case Header::Inertia:
                kobuki_inertia.deserialise(data_buffer);
                sig_inertia.emit();
                break;
              case Header::Cliff:
                kobuki_cliff.deserialise(data_buffer);
                sig_cliff.emit();
                break;
              case Header::Current:
                kobuki_current.deserialise(data_buffer);
                sig_current.emit();
                break;
              case Header::Gpio:
                kobuki_gp_input.deserialise(data_buffer);
                sig_gp_input.emit();
                break;
              // the rest are services
              case Header::InfraRed:
                kobuki_ir.deserialise(data_buffer);
                sig_ir.emit();
                break;
              case Header::Time:
                kobuki_time.deserialise(data_buffer);
                sig_time.emit();
                break;
              case Header::Hardware:
                kobuki_hw.deserialise(data_buffer);
                sig_hw.emit();
                break;
              case Header::Firmware:
                kobuki_fw.deserialise(data_buffer);
                sig_fw.emit();
                break;
              case Header::Eeprom:
                kobuki_eeprom.deserialise(data_buffer);
                sig_eeprom.emit();
                break;
              default:
                std::cout << "unexpected case reached. flushing current buffer." << std::endl;
                data_buffer.clear();
                break;
            }
          }
        }
        get_packet = true;
      }
    }

    if (get_packet) {
      sendBaseControlCommand();
    } // send the command packet to mainboard;
  }
}

void Kobuki::getCoreSensorData(kobuki_comms::CoreSensors &sensor_data)
{
  if (protocol_version == "2.0") {
    sensor_data = kobuki_default.data;
  }
}

void Kobuki::getIRData(kobuki_comms::IR &data)
{
  if (protocol_version == "2.0")
    data = kobuki_ir.data;
}

void Kobuki::getDockIRData(kobuki_comms::DockIR &data)
{
  if (protocol_version == "2.0")
    data = kobuki_dock_ir.data;
}

ecl::Angle<double> Kobuki::getHeading() const {
  ecl::Angle<double> heading;
  if ( simulation ) {
    heading = kobuki_sim.heading;
  } else {
    // raw data angles are in hundredths of a degree, convert to radians.
    heading = (static_cast<double>(kobuki_inertia.data.angle) / 100.0) * ecl::pi /180.0;
  }
  return heading;
}

void Kobuki::getCliffData(kobuki_comms::Cliff &data)
{
  if (protocol_version == "2.0")
    data = kobuki_cliff.data;
}

void Kobuki::getCurrentData(kobuki_comms::Current &data)
{
  if (protocol_version == "2.0")
    data = kobuki_current.data;
}

void Kobuki::getHWData(kobuki_comms::HW &data)
{
  if (protocol_version == "2.0")
    data = kobuki_hw.data;
}

void Kobuki::getFWData(kobuki_comms::FW &data)
{
  if (protocol_version == "2.0")
    data = kobuki_fw.data;
}

void Kobuki::getTimeData(kobuki_comms::Time &data)
{
  if (protocol_version == "2.0")
    data = kobuki_time.data;
}

void Kobuki::getEEPROMData(kobuki_comms::EEPROM &data)
{
  if (protocol_version == "2.0")
    data = kobuki_eeprom.data;
}

void Kobuki::getGpInputData(kobuki_comms::GpInput &data)
{
  if (protocol_version == "2.0")
    data = kobuki_gp_input.data;
}
void Kobuki::resetOdometry() {
  if ( simulation ) {
    kobuki_sim.reset();
  }
  last_rad_left = 0.0;
  last_rad_right = 0.0;
  last_velocity_left = 0.0;
  last_velocity_right = 0.0;

  imu_heading_offset = kobuki_inertia.data.angle;
}

void Kobuki::getWheelJointStates(double &wheel_left_angle, double &wheel_left_angle_rate,
                          double &wheel_right_angle, double &wheel_right_angle_rate) {

  if ( simulation ) {
    wheel_left_angle = kobuki_sim.left_wheel_angle;
    wheel_right_angle = kobuki_sim.right_wheel_angle;
    wheel_left_angle_rate = kobuki_sim.left_wheel_angle_rate;
    wheel_right_angle_rate = kobuki_sim.right_wheel_angle_rate;
  } else {
    wheel_left_angle = last_rad_left;
    wheel_right_angle = last_rad_right;
    wheel_left_angle_rate = last_velocity_left;
    wheel_right_angle_rate = last_velocity_right;
  }
}
void Kobuki::updateOdometry(ecl::Pose2D<double> &pose_update,
                            ecl::linear_algebra::Vector3d &pose_update_rates) {
  if ( simulation ) {
    pose_update = kinematics->forward(kobuki_sim.left_wheel_angle_update, kobuki_sim.right_wheel_angle_update);
    // should add pose_update_rates here as well.
  } else {
    static bool init_l = false;
    static bool init_r = false;
    double left_diff_ticks = 0.0f;
    double right_diff_ticks = 0.0f;
    unsigned short curr_tick_left = 0;
    unsigned short curr_tick_right = 0;
    unsigned short curr_timestamp = 0;
    curr_timestamp = kobuki_default.data.time_stamp;
    curr_tick_left = kobuki_default.data.left_encoder;
    if (!init_l)
    {
      last_tick_left = curr_tick_left;
      init_l = true;
    }
    left_diff_ticks = (double)(short)((curr_tick_left - last_tick_left) & 0xffff);
    last_tick_left = curr_tick_left;
    last_rad_left += tick_to_rad * left_diff_ticks;
    last_mm_left += tick_to_mm / 1000.0f * left_diff_ticks;

    curr_tick_right = kobuki_default.data.right_encoder;
    if (!init_r)
    {
      last_tick_right = curr_tick_right;
      init_r = true;
    }
    right_diff_ticks = (double)(short)((curr_tick_right - last_tick_right) & 0xffff);
    last_tick_right = curr_tick_right;
    last_rad_right += tick_to_rad * right_diff_ticks;
    last_mm_right += tick_to_mm / 1000.0f * right_diff_ticks;

    // TODO this line and the last statements are really ugly; refactor, put in another place
    pose_update = kinematics->forward(tick_to_rad * left_diff_ticks, tick_to_rad * right_diff_ticks);

    if (curr_timestamp != last_timestamp)
    {
      last_diff_time = ((double)(short)((curr_timestamp - last_timestamp) & 0xffff)) / 1000.0f;
      last_timestamp = curr_timestamp;
      last_velocity_left = (tick_to_rad * left_diff_ticks) / last_diff_time;
      last_velocity_right = (tick_to_rad * right_diff_ticks) / last_diff_time;
    } else {
      // we need to set the last_velocity_xxx to zero?
    }

    pose_update_rates << pose_update.x()/last_diff_time,
                         pose_update.y()/last_diff_time,
                         pose_update.heading()/last_diff_time;
  }
}

/*****************************************************************************
** Commands
*****************************************************************************/
void Kobuki::toggleLed(const enum LedNumber &number, const enum LedColour &colour) {
  kobuki_command.update(number,colour);
  Command outgoing = kobuki_command;
  outgoing.data.command = Command::SetDigitalOut;
  sendCommand(outgoing);
}

void Kobuki::setBaseControlCommand(double vx, double wz)
{
  if (wz == 0.0f)
    radius = 0;
  else if (vx == 0.0f && wz > 0.0f)
    radius = 1;
  else if (vx == 0.0f && wz < 0.0f)
    radius = -1;
  else
    radius = (short)(vx * 1000.0f / wz);

  speed = (short)(1000.0f * std::max(vx + bias * wz / 2.0f, vx - bias * wz / 2.0f));

  if ( simulation ) {
    kobuki_sim.velocity = vx;
    kobuki_sim.angular_velocity = wz;
  }
}

void Kobuki::sendBaseControlCommand()
{
  if ( !simulation ) {
    //std::cout << "speed = " << speed << ", radius = " << radius << std::endl;
    unsigned char cmd[] = {0xaa, 0x55, 5, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char cs(0);

    union_sint16 union_speed, union_radius;
    union_speed.word = speed;
    union_radius.word = radius;

    cmd[4] = union_speed.byte[0];
    cmd[5] = union_speed.byte[1];
    cmd[6] = union_radius.byte[0];
    cmd[7] = union_radius.byte[1];

    //memcpy(cmd + 4, &speed,  sizeof(short) );
    //memcpy(cmd + 6, &radius, sizeof(short) );

    for (int i = 2; i <= 6; i++)
      cs ^= cmd[i];
    cmd[8] = cs;

    serial.write(cmd, 9);
  }
}

void Kobuki::sendCommand(Command &command)
{
  if ( !simulation ) {
    command_buffer.clear();
    command_buffer.resize(64);
    command_buffer.push_back(0xaa);
    command_buffer.push_back(0x55);
    command_buffer.push_back(0); // size of payload only, not stx, not etx, not length

    if (!command.serialise(command_buffer))
    {
      sig_error.emit("command serialise failed.");
    }
    command_buffer[2] = command_buffer.size() - 3;
    unsigned char checksum = 0;
    for (unsigned int i = 2; i < command_buffer.size(); i++)
      checksum ^= (command_buffer[i]);

    command_buffer.push_back(checksum);
    serial.write(&command_buffer[0], command_buffer.size());

//    for (unsigned int i = 0; i < command_buffer.size(); ++i)
//    {
//      std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << (unsigned)command_buffer[i] << std::dec
//          << std::setfill(' ') << " ";
//    }
//    std::cout << std::endl;

    if (command.data.command == Command::BaseControl)
    {
      radius = command.data.radius;
      speed = command.data.speed;
    }
  }
}

bool Kobuki::enable()
{
//	is_running = true;
  is_enabled = true;
  return true;
}

bool Kobuki::disable()
{
  setBaseControlCommand(0.0f, 0.0f);
  sendBaseControlCommand();
//	is_running = false;
  is_enabled = false;
  return true;
}

} // namespace kobuki