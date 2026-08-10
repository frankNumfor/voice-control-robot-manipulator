#include "arduinobot_controller/arduinobot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <thread>
#include <chrono>

namespace arduinobot_controller
{

std::string compensateZeros(int value)
{
  std::string compensate_zeros = "";
  if(value < 10){
    compensate_zeros = "00";
  } else if(value < 100){
    compensate_zeros = "0";
  } else{
    compensate_zeros = "";
  }
  return compensate_zeros;
}

ArduinobotInterface::ArduinobotInterface()
{
}

ArduinobotInterface::~ArduinobotInterface()
{
  if(arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch(...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Something went wrong while closing the connection with port " << port_);
    }
  }
}

CallbackReturn ArduinobotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

  if(result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch(const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ArduinobotInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  position_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  prev_position_commands_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> ArduinobotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for(size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    if(info_.joints[i].name == "joint_4")
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinobotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for(size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn ArduinobotInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Starting the robot hardware...");
  position_commands_ = {0.0, 0.0, 0.0, 0.0};
  prev_position_commands_ = {0.0, 0.0, 0.0, 0.0};
  position_states_ = {0.0, 0.0, 0.0, 0.0};
  velocity_states_ = {0.0, 0.0, 0.0, 0.0};

  const int max_attempts = 5;
  bool connected = false;

  for (int attempt = 1; attempt <= max_attempts && !connected; attempt++)
  {
    try
    {
      arduino_.Open(port_);
      arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      connected = true;
    }
    catch (...)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("ArduinobotInterface"),
        "Attempt " << attempt << "/" << max_attempts << " failed to open port " << port_ << ". Retrying...");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  if (!connected)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Failed to open port " << port_ << " after " << max_attempts << " attempts.");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}



CallbackReturn ArduinobotInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Stopping the robot hardware...");
  if(arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch(...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Something went wrong while closing the connection with the port " << port_);
      return CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinobotInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  position_states_ = position_commands_;
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type ArduinobotInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  if (position_commands_ == prev_position_commands_)
  {
    // Nothing changed, do not send any command
    return hardware_interface::return_type::OK;
  }

  std::string msg;
  int base = static_cast<int>(((position_commands_.at(0) + (M_PI / 2)) * 180) / M_PI);
  msg.append("b");
  msg.append(compensateZeros(base));
  msg.append(std::to_string(base));
  msg.append(",");
  int shoulder = 180 - static_cast<int>(((position_commands_.at(1) + (M_PI / 2)) * 180) / M_PI);
  msg.append("s");
  msg.append(compensateZeros(shoulder));
  msg.append(std::to_string(shoulder));
  msg.append(",");
  int elbow = static_cast<int>(((position_commands_.at(2) + (M_PI / 2)) * 180) / M_PI);
  msg.append("e");
  msg.append(compensateZeros(elbow));
  msg.append(std::to_string(elbow));
  msg.append(",");
  int gripper = static_cast<int>((-position_commands_.at(3) * 180) / (M_PI / 2));
  msg.append("g");
  msg.append(compensateZeros(gripper));
  msg.append(std::to_string(gripper));
  msg.append(",");

  try
  {
    arduino_.Write(msg);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Exception while sending message " << msg << " to port " << port_ << ": " << e.what());
    return hardware_interface::return_type::ERROR;
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Unknown exception while sending message " << msg << " to port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  prev_position_commands_ = position_commands_;
  return hardware_interface::return_type::OK;
}

}  // namespace arduinobot_controller

PLUGINLIB_EXPORT_CLASS(arduinobot_controller::ArduinobotInterface, hardware_interface::SystemInterface)
