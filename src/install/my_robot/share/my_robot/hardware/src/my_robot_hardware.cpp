// #include "my_robot/my_robot_hardware.hpp"

// namespace my_robot
// {
//     // Define the hardware interface constants if they are not defined
//     const std::string HW_IF_POSITION = "position";
//     const std::string HW_IF_VELOCITY = "velocity";

//     hardware_interface::CallbackReturn MyRobotHardware::on_init(const hardware_interface::HardwareInfo &info)
//     {
//         if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
//         {
//             return hardware_interface::CallbackReturn::ERROR;
//         }

//         number_of_joints_ = info.joints.size();
//         hw_positions_.resize(number_of_joints_, 0.0);
//         hw_velocities_.resize(number_of_joints_, 0.0);
//         hw_commands_.resize(number_of_joints_, 0.0);
//         joint_names_.resize(number_of_joints_);

//         for (size_t i = 0; i < number_of_joints_; ++i)
//         {
//             joint_names_[i] = info.joints[i].name;
//         }

//         // Initialize pigpio
//         pi_ = pigpio_start(nullptr, nullptr);
//         if (pi_ < 0)
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Failed to initialize pigpio");
//             return hardware_interface::CallbackReturn::ERROR;
//         }

//         // Define motor control pins
//         L_IN1 = 20;
//         L_IN2 = 21;
//         L_PWM1 = 0;
//         L_IN3 = 22;
//         L_IN4 = 23;
//         L_PWM2 = 1;
//         R_IN1 = 24;
//         R_IN2 = 25;
//         R_PWM1 = 12;
//         R_IN3 = 26;
//         R_IN4 = 27;
//         R_PWM2 = 13;

//         // Set up motor control pins
//         set_mode(pi_, L_IN1, PI_OUTPUT);
//         set_mode(pi_, L_IN2, PI_OUTPUT);
//         set_mode(pi_, L_PWM1, PI_OUTPUT);
//         set_mode(pi_, L_IN3, PI_OUTPUT);
//         set_mode(pi_, L_IN4, PI_OUTPUT);
//         set_mode(pi_, L_PWM2, PI_OUTPUT);
//         set_mode(pi_, R_IN1, PI_OUTPUT);
//         set_mode(pi_, R_IN2, PI_OUTPUT);
//         set_mode(pi_, R_PWM1, PI_OUTPUT);
//         set_mode(pi_, R_IN3, PI_OUTPUT);
//         set_mode(pi_, R_IN4, PI_OUTPUT);
//         set_mode(pi_, R_PWM2, PI_OUTPUT);

//         RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface initialized successfully.");
//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::CallbackReturn MyRobotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
//     {
//         std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
//         RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface activated successfully.");
//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::CallbackReturn MyRobotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface deactivated successfully.");
//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::return_type MyRobotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
//     {
//         for (std::size_t i = 0; i < hw_velocities_.size(); i++)
//         {
//             hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
//             RCLCPP_INFO(
//                 rclcpp::get_logger("MyRobotHardware"),
//                 "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
//                 hw_velocities_[i], joint_names_[i].c_str());
//         }

//         return hardware_interface::return_type::OK;
//     }

//     hardware_interface::return_type MyRobotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
//     {
//         int left_front_speed = static_cast<int>(hw_commands_[0] * 255);  // Scale command to PWM range
//         int left_rear_speed = static_cast<int>(hw_commands_[1] * 255);   // Scale command to PWM range
//         int right_front_speed = static_cast<int>(hw_commands_[2] * 255); // Scale command to PWM range
//         int right_rear_speed = static_cast<int>(hw_commands_[3] * 255);  // Scale command to PWM range

//         hw_velocities_[0] = hw_commands_[0];
//         hw_velocities_[1] = hw_commands_[1];
//         hw_velocities_[2] = hw_commands_[2];
//         hw_velocities_[3] = hw_commands_[3];

//         // Left front motor control
//         if (left_front_speed >= 0)
//         {
//             gpio_write(pi_, L_IN1, PI_LOW);
//             gpio_write(pi_, L_IN2, PI_HIGH);
//         }
//         else
//         {
//             gpio_write(pi_, L_IN1, PI_HIGH);
//             gpio_write(pi_, L_IN2, PI_LOW);
//         }
//         set_PWM_dutycycle(pi_, L_PWM1, abs(left_front_speed));

//         // Left rear motor control
//         if (left_rear_speed >= 0)
//         {
//             gpio_write(pi_, L_IN3, PI_HIGH);
//             gpio_write(pi_, L_IN4, PI_LOW);
//         }
//         else
//         {
//             gpio_write(pi_, L_IN3, PI_LOW);
//             gpio_write(pi_, L_IN4, PI_HIGH);
//         }
//         set_PWM_dutycycle(pi_, L_PWM2, abs(left_rear_speed));

//         // Right front motor control
//         if (right_front_speed >= 0)
//         {
//             gpio_write(pi_, R_IN1, PI_HIGH);
//             gpio_write(pi_, R_IN2, PI_LOW);
//         }
//         else
//         {
//             gpio_write(pi_, R_IN1, PI_LOW);
//             gpio_write(pi_, R_IN2, PI_HIGH);
//         }
//         set_PWM_dutycycle(pi_, R_PWM1, abs(right_front_speed));

//         // Right rear motor control
//         if (right_rear_speed >= 0)
//         {
//             gpio_write(pi_, R_IN3, PI_LOW);
//             gpio_write(pi_, R_IN4, PI_HIGH);
//         }
//         else
//         {
//             gpio_write(pi_, R_IN3, PI_HIGH);
//             gpio_write(pi_, R_IN4, PI_LOW);
//         }
//         set_PWM_dutycycle(pi_, R_PWM2, abs(right_rear_speed));

//         return hardware_interface::return_type::OK;
//     }

//     std::vector<hardware_interface::StateInterface> MyRobotHardware::export_state_interfaces()
//     {
//         std::vector<hardware_interface::StateInterface> state_interfaces;

//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[0], HW_IF_POSITION, &hw_positions_[0]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[1], HW_IF_POSITION, &hw_positions_[1]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[2], HW_IF_POSITION, &hw_positions_[2]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[3], HW_IF_POSITION, &hw_positions_[3]));

//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[0], HW_IF_VELOCITY, &hw_velocities_[0]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[1], HW_IF_VELOCITY, &hw_velocities_[1]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[2], HW_IF_VELOCITY, &hw_velocities_[2]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[3], HW_IF_VELOCITY, &hw_velocities_[3]));

//         return state_interfaces;
//     }

//     std::vector<hardware_interface::CommandInterface> MyRobotHardware::export_command_interfaces()
//     {
//         std::vector<hardware_interface::CommandInterface> command_interfaces;

//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[0], HW_IF_VELOCITY, &hw_commands_[0]));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[1], HW_IF_VELOCITY, &hw_commands_[1]));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[2], HW_IF_VELOCITY, &hw_commands_[2]));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[3], HW_IF_VELOCITY, &hw_commands_[3]));

//         return command_interfaces;
//     }

// } // namespace my_robot

// // Register the hardware interface as a plugin
// #include "pluginlib/class_list_macros.hpp"

// PLUGINLIB_EXPORT_CLASS(my_robot::MyRobotHardware, hardware_interface::SystemInterface)


// END OF THE FIRST ONE
 
// #include "my_robot/my_robot_hardware.hpp"

// namespace my_robot
// {
//     // Define the hardware interface constants if they are not defined
//     const std::string HW_IF_POSITION = "position";
//     const std::string HW_IF_VELOCITY = "velocity";

//     hardware_interface::CallbackReturn MyRobotHardware::on_init(const hardware_interface::HardwareInfo &info)
//     {
//         if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
//         {
//             return hardware_interface::CallbackReturn::ERROR;
//         }

//         number_of_joints_ = info.joints.size();
//         hw_positions_.resize(number_of_joints_, 0.0);
//         hw_velocities_.resize(number_of_joints_, 0.0);
//         hw_commands_.resize(number_of_joints_, 0.0);
//         joint_names_.resize(number_of_joints_);

//         for (size_t i = 0; i < number_of_joints_; ++i)
//         {
//             joint_names_[i] = info.joints[i].name;
//         }

//         // Initialize pigpio
//         pi_ = pigpio_start(nullptr, nullptr);
//         if (pi_ < 0)
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Failed to initialize pigpio");
//             return hardware_interface::CallbackReturn::ERROR;
//         }

//         // Define motor control pins
//         L_IN1 = 20;
//         L_IN2 = 21;
//         L_PWM1 = 0;
//         L_IN3 = 22;
//         L_IN4 = 23;
//         L_PWM2 = 1;
//         R_IN1 = 24;
//         R_IN2 = 25;
//         R_PWM1 = 12;
//         R_IN3 = 26;
//         R_IN4 = 27;
//         R_PWM2 = 13;

//         // Set up motor control pins
//         set_mode(pi_, L_IN1, PI_OUTPUT);
//         set_mode(pi_, L_IN2, PI_OUTPUT);
//         set_mode(pi_, L_PWM1, PI_OUTPUT);
//         set_mode(pi_, L_IN3, PI_OUTPUT);
//         set_mode(pi_, L_IN4, PI_OUTPUT);
//         set_mode(pi_, L_PWM2, PI_OUTPUT);
//         set_mode(pi_, R_IN1, PI_OUTPUT);
//         set_mode(pi_, R_IN2, PI_OUTPUT);
//         set_mode(pi_, R_PWM1, PI_OUTPUT);
//         set_mode(pi_, R_IN3, PI_OUTPUT);
//         set_mode(pi_, R_IN4, PI_OUTPUT);
//         set_mode(pi_, R_PWM2, PI_OUTPUT);

//         RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface initialized successfully.");
//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::CallbackReturn MyRobotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
//     {
//         std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
//         RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface activated successfully.");
//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::CallbackReturn MyRobotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface deactivated successfully.");
//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::return_type MyRobotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)

//     {
//         // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
//         for (std::size_t i = 0; i < hw_velocities_.size(); i++)
//         {
//             // Simulate DiffBot wheels's movement as a first-order system
//             // Update the joint status: this is a revolute joint without any limit.
//             // Simply integrates
//             hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

//             RCLCPP_INFO(
//                 rclcpp::get_logger("MyRobotHardware"),
//                 "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
//                 hw_velocities_[i], joint_names_[i].c_str());
//         }
//         // END: This part here is for exemplary purposes - Please do not copy to your production code

//         return hardware_interface::return_type::OK;
//     }

//     hardware_interface::return_type MyRobotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
//     {
//         // Write commands to hardware here using pigpio
//         int left_front_speed = static_cast<int>(hw_commands_[0] * 255);  // Scale command to PWM range
//         int left_rear_speed = static_cast<int>(hw_commands_[1] * 255);   // Scale command to PWM range
//         int right_front_speed = static_cast<int>(hw_commands_[2] * 255); // Scale command to PWM range
//         int right_rear_speed = static_cast<int>(hw_commands_[3] * 255);  // Scale command to PWM range

//         hw_velocities_[0] = hw_commands_[0];
//         hw_velocities_[1] = hw_commands_[1];
//         hw_velocities_[2] = hw_commands_[2];
//         hw_velocities_[3] = hw_commands_[3];

//         // Left front motor control
//         if (left_front_speed >= 0)
//         {
//             gpio_write(pi_, L_IN1, 0);
//             gpio_write(pi_, L_IN2, 1);
//         }
//         else
//         {
//             gpio_write(pi_, L_IN1, 1);
//             gpio_write(pi_, L_IN2, 0);
//         }
//         set_PWM_dutycycle(pi_, L_PWM1, abs(left_front_speed));

//         // Left rear motor control
//         if (left_rear_speed >= 0)
//         {
//             gpio_write(pi_, L_IN3, 0);
//             gpio_write(pi_, L_IN4, 1);
//         }
//         else
//         {
//             gpio_write(pi_, L_IN3, 1);
//             gpio_write(pi_, L_IN4, 0);
//         }
//         set_PWM_dutycycle(pi_, L_PWM2, abs(left_rear_speed));

//         // Right front motor control
//         if (right_front_speed >= 0)
//         {
//             gpio_write(pi_, R_IN1, 1);
//             gpio_write(pi_, R_IN2, 0);
//         }
//         else
//         {
//             gpio_write(pi_, R_IN1, 0);
//             gpio_write(pi_, R_IN2, 1);
//         }
//         set_PWM_dutycycle(pi_, R_PWM1, abs(right_front_speed));

//         // Right rear motor control
//         if (right_rear_speed >= 0)
//         {
//             gpio_write(pi_, R_IN3, 1);
//             gpio_write(pi_, R_IN4, 0);
//         }
//         else
//         {
//             gpio_write(pi_, R_IN3, 0);
//             gpio_write(pi_, R_IN4, 1);
//         }
//         set_PWM_dutycycle(pi_, R_PWM2, abs(right_rear_speed));

//         return hardware_interface::return_type::OK;
//     }

//     std::vector<hardware_interface::StateInterface> MyRobotHardware::export_state_interfaces()
//     {
//         std::vector<hardware_interface::StateInterface> state_interfaces;

//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[0], HW_IF_POSITION, &hw_positions_[0]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[1], HW_IF_POSITION, &hw_positions_[1]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[2], HW_IF_POSITION, &hw_positions_[2]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[3], HW_IF_POSITION, &hw_positions_[3]));

//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[0], HW_IF_VELOCITY, &hw_velocities_[0]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[1], HW_IF_VELOCITY, &hw_velocities_[1]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[2], HW_IF_VELOCITY, &hw_velocities_[2]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[3], HW_IF_VELOCITY, &hw_velocities_[3]));

//         return state_interfaces;
//     }

//     std::vector<hardware_interface::CommandInterface> MyRobotHardware::export_command_interfaces()
//     {
//         std::vector<hardware_interface::CommandInterface> command_interfaces;

//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[0], HW_IF_VELOCITY, &hw_commands_[0]));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[1], HW_IF_VELOCITY, &hw_commands_[1]));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[2], HW_IF_VELOCITY, &hw_commands_[2]));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[3], HW_IF_VELOCITY, &hw_commands_[3]));

//         return command_interfaces;
//     }

// } // namespace my_robot

// // Register the hardware interface as a plugin
// #include "pluginlib/class_list_macros.hpp"

// PLUGINLIB_EXPORT_CLASS(my_robot::MyRobotHardware, hardware_interface::SystemInterface)

// MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAIN MAINMAIN MAIN MAIN MAIN 

// #include "my_robot/my_robot_hardware.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "hardware_interface/lexical_casts.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"

// namespace my_robot
// {

//     hardware_interface::CallbackReturn MyRobotHardware::on_init(const hardware_interface::HardwareInfo & info)
//     {
//         if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
//         {
//             return hardware_interface::CallbackReturn::ERROR;
//         }
//         for (const hardware_interface::ComponentInfo &joint : info_.joints)
//         {
//             // DiffBotSystem has exactly two states and one command interface on each joint
//             if (joint.command_interfaces.size() != 1)
//             {
//                 RCLCPP_FATAL(
//                     rclcpp::get_logger("DiffBotSystemHardware"),
//                     "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
//                     joint.command_interfaces.size());
//                 return hardware_interface::CallbackReturn::ERROR;
//             }

//             if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
//             {
//                 RCLCPP_FATAL(
//                     rclcpp::get_logger("DiffBotSystemHardware"),
//                     "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
//                     joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
//                 return hardware_interface::CallbackReturn::ERROR;
//             }

//             if (joint.state_interfaces.size() != 2)
//             {
//                 RCLCPP_FATAL(
//                     rclcpp::get_logger("DiffBotSystemHardware"),
//                     "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
//                     joint.state_interfaces.size());
//                 return hardware_interface::CallbackReturn::ERROR;
//             }

//             if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
//             {
//                 RCLCPP_FATAL(
//                     rclcpp::get_logger("DiffBotSystemHardware"),
//                     "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
//                     joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
//                 return hardware_interface::CallbackReturn::ERROR;
//             }

//             if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
//             {
//                 RCLCPP_FATAL(
//                     rclcpp::get_logger("DiffBotSystemHardware"),
//                     "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
//                     joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
//                 return hardware_interface::CallbackReturn::ERROR;
//             }
//         }

//         number_of_joints_ = info.joints.size();
//         hw_positions_.resize(number_of_joints_, 0.0);
//         hw_velocities_.resize(number_of_joints_, 0.0);
//         hw_commands_.resize(number_of_joints_, 0.0);
//         joint_names_.resize(number_of_joints_);
        


//         for (size_t i = 0; i < number_of_joints_; ++i)
//         {
//             joint_names_[i] = info.joints[i].name;
//         }

//         // Initialize pigpio
//         pi_ = pigpio_start(nullptr, nullptr);
//         if (pi_ < 0)
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Failed to initialize pigpio");
//             return hardware_interface::CallbackReturn::ERROR;
//         }

//         // Define motor control pins
//         L_IN4 = 23;
//         L_PWM2 = 1;
//         L_IN3 = 22;
//         L_IN1 = 20;
//         L_IN2 = 21;
//         L_PWM1 = 0;
        

//         R_IN1 = 24;
//         R_IN2 = 25;
//         R_PWM1 = 12;
//         R_IN3 = 26;
//         R_IN4 = 27;
//         R_PWM2 = 13;

//         // Set up motor control pins
//         set_mode(pi_, L_IN1, PI_OUTPUT);
//         set_mode(pi_, L_IN2, PI_OUTPUT);
//         set_mode(pi_, L_PWM1, PI_OUTPUT);
//         set_mode(pi_, L_IN3, PI_OUTPUT);
//         set_mode(pi_, L_IN4, PI_OUTPUT);
//         set_mode(pi_, L_PWM2, PI_OUTPUT);
//         set_mode(pi_, R_IN1, PI_OUTPUT);
//         set_mode(pi_, R_IN2, PI_OUTPUT);
//         set_mode(pi_, R_PWM1, PI_OUTPUT);
//         set_mode(pi_, R_IN3, PI_OUTPUT);
//         set_mode(pi_, R_IN4, PI_OUTPUT);
//         set_mode(pi_, R_PWM2, PI_OUTPUT);

//         RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface initialized successfully.");
//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     std::vector<hardware_interface::StateInterface> MyRobotHardware::export_state_interfaces()
//     {
//         std::vector<hardware_interface::StateInterface> state_interfaces;

//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[0], hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[1], hardware_interface::HW_IF_POSITION, &hw_positions_[1]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[2], hardware_interface::HW_IF_POSITION, &hw_positions_[2]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[3], hardware_interface::HW_IF_POSITION, &hw_positions_[3]));

//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[0], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[1], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[2], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[2]));
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//             joint_names_[3], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[3]));

//         return state_interfaces;
//     }

//     std::vector<hardware_interface::CommandInterface> MyRobotHardware::export_command_interfaces()
//     {
//         std::vector<hardware_interface::CommandInterface> command_interfaces;

//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[0], hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[1], hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[2], hardware_interface::HW_IF_VELOCITY, &hw_commands_[2]));
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//             joint_names_[3], hardware_interface::HW_IF_VELOCITY, &hw_commands_[3]));

//         return command_interfaces;
//     }

//     hardware_interface::CallbackReturn MyRobotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
//     {
//         std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
//         RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface activated successfully.");
//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::CallbackReturn MyRobotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
//     {
//         gpio_write(pi_, L_IN1, 0);
//         gpio_write(pi_, L_IN2, 0);
//         gpio_write(pi_, L_IN3, 0);
//         gpio_write(pi_, L_IN4, 0);
//         gpio_write(pi_, R_IN1, 0);
//         gpio_write(pi_, R_IN2, 0);
//         gpio_write(pi_, R_IN3, 0);
//         gpio_write(pi_, R_IN4, 0);

//         RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface deactivated successfully.");
//         return hardware_interface::CallbackReturn::SUCCESS;
//     }

//     hardware_interface::return_type MyRobotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
//     {
//         for (std::size_t i = 0; i < hw_velocities_.size(); i++)
//         {
//             hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

//             RCLCPP_INFO(
//                 rclcpp::get_logger("MyRobotHardware"),
//                 "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
//                 hw_velocities_[i], joint_names_[i].c_str());
//         }
//         return hardware_interface::return_type::OK;
//     }

//     hardware_interface::return_type MyRobotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
//     {

//         // Write commands to hardware here using pigpio
//         int left_front_speed = static_cast<int>(hw_commands_[0] * 100);  // Scale command to PWM range
//         int left_rear_speed = static_cast<int>(hw_commands_[1] * 100);   // Scale command to PWM range
//         int right_front_speed = static_cast<int>(hw_commands_[2] * 100); // Scale command to PWM range
//         int right_rear_speed = static_cast<int>(hw_commands_[3] * 100);  // Scale command to PWM range

//         hw_velocities_[0] = hw_commands_[0];
//         hw_velocities_[1] = hw_commands_[1];
//         hw_velocities_[2] = hw_commands_[2];
//         hw_velocities_[3] = hw_commands_[3];

//         // Left front motor control

//         // Left front motor control
//         // if (left_front_speed >= 0)
//         // {
//         //     gpio_write(pi_, L_IN1, PI_HIGH);
//         //     gpio_write(pi_, L_IN2, PI_LOW);
//         // }
//         // else
//         // {
//         //     gpio_write(pi_, L_IN1, PI_LOW);
//         //     gpio_write(pi_, L_IN2, PI_HIGH);
//         // }
//         gpio_write(pi_, L_IN1, PI_HIGH);
//         gpio_write(pi_, L_IN2, PI_LOW);
//         set_PWM_dutycycle(pi_, L_PWM1, abs(100));

//         // Left rear motor control
//         if (left_rear_speed >= 0)
//         {
//             gpio_write(pi_, L_IN3, PI_HIGH);
//             gpio_write(pi_, L_IN4, PI_LOW);
//         }
//         else
//         {
//             gpio_write(pi_, L_IN3, PI_LOW);
//             gpio_write(pi_, L_IN4, PI_HIGH);
//         }
//         set_PWM_dutycycle(pi_, L_PWM2, abs(left_rear_speed));
//         // Right front motor control
//         if (right_front_speed >= 0)
//         {
//             gpio_write(pi_, R_IN1, PI_HIGH);
//             gpio_write(pi_, R_IN2, PI_LOW);
//         }
//         else
//         {
//             gpio_write(pi_, R_IN1, PI_LOW);
//             gpio_write(pi_, R_IN2, PI_HIGH);
//         }
//         set_PWM_dutycycle(pi_, R_PWM1, abs(right_front_speed));

//         // Right rear motor control
//         if (right_rear_speed >= 0)
//         {
//             gpio_write(pi_, R_IN3, PI_HIGH);
//             gpio_write(pi_, R_IN4, PI_LOW);
//         }
//         else
//         {
//             gpio_write(pi_, R_IN3, PI_LOW);
//             gpio_write(pi_, R_IN4, PI_HIGH);
//         }
//         set_PWM_dutycycle(pi_, R_PWM2, abs(right_rear_speed));

//         RCLCPP_INFO(
//             rclcpp::get_logger("MyRobotHardware"),
//             "Commands: LF: %d, LR: %d, RF: %d, RR: %d",
//             left_front_speed, left_rear_speed, right_front_speed, right_rear_speed);

//         return hardware_interface::return_type::OK;
//     }

// } // namespace my_robot

// #include "pluginlib/class_list_macros.hpp"

// PLUGINLIB_EXPORT_CLASS(my_robot::MyRobotHardware, hardware_interface::SystemInterface)

///////////////////////////////////////////////////////////////////////////////////////////

#include "my_robot/my_robot_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <algorithm>

namespace my_robot
{

    hardware_interface::CallbackReturn MyRobotHardware::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        number_of_joints_ = info.joints.size();
        hw_positions_.resize(number_of_joints_, 0.0);
        hw_velocities_.resize(number_of_joints_, 0.0);
        hw_commands_.resize(number_of_joints_, 0.0);
        joint_names_.resize(number_of_joints_);

        for (size_t i = 0; i < number_of_joints_; ++i)
        {
            joint_names_[i] = info.joints[i].name;
        }

        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Failed to initialize pigpio");
            return hardware_interface::CallbackReturn::ERROR;
        }

        
        L_IN1 = 24;
        L_IN2 = 25;
        L_PWM1 = 12;

        R_IN1 = 26;
        R_IN2 = 27;
        R_PWM1 = 13;
        

        set_mode(pi_, L_IN1, PI_OUTPUT);
        set_mode(pi_, L_IN2, PI_OUTPUT);
        set_mode(pi_, L_PWM1, PI_OUTPUT);

        set_mode(pi_, R_IN1, PI_OUTPUT);
        set_mode(pi_, R_IN2, PI_OUTPUT);
        set_mode(pi_, R_PWM1, PI_OUTPUT);

        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface initialized successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MyRobotHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[0], hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[1], hardware_interface::HW_IF_POSITION, &hw_positions_[1]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[0], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[1], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MyRobotHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[0], hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[1], hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]));
        return command_interfaces;
    }

    hardware_interface::CallbackReturn MyRobotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface activated successfully.");

        for (size_t i = 0; i < number_of_joints_; ++i)
        {
            RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Initial command for joint %s: %f", joint_names_[i].c_str(), hw_commands_[i]);
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MyRobotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        gpio_write(pi_, L_IN1, 0);
        gpio_write(pi_, L_IN2, 0);
        gpio_write(pi_, L_PWM1, 0);

        gpio_write(pi_, R_IN1, 0);
        gpio_write(pi_, R_IN2, 0);
        gpio_write(pi_, R_PWM1, 0);

        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Hardware interface deactivated successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MyRobotHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        for (std::size_t i = 0; i < hw_velocities_.size(); i++)
        {
            hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

            // RCLCPP_INFO(
            //     rclcpp::get_logger("MyRobotHardware"),
            //     "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
            //     hw_velocities_[i], joint_names_[i].c_str());
        }
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type MyRobotHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        float left_velocity = static_cast<float>(hw_commands_[0]); // converting to m/s
        float right_velocity = static_cast<float>(hw_commands_[1]);

        hw_velocities_[0] = hw_commands_[0] * 0.20;
        hw_velocities_[1] = hw_commands_[1] * 0.20;

        // Log the received commands for debugging
        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Received commands: LF: %f, RF: %f", left_velocity, right_velocity);

        float linear_max_vel = 20.9;
        float angular_max_vel = 1;
        float combined_max_vel = 21.9;

        if (abs(left_velocity) == abs(right_velocity)) {
            if ((left_velocity > 0 && right_velocity > 0) || (left_velocity < 0 && right_velocity < 0)) {
                left_velocity/=linear_max_vel;
                right_velocity/=linear_max_vel;

            } else if ((left_velocity > 0 && right_velocity < 0) || (left_velocity < 0 && right_velocity > 0)) {
                left_velocity/=1;
                right_velocity/=1;
            }
        } else {
            left_velocity/=combined_max_vel;
            right_velocity/=combined_max_vel;
        }

        // RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Left Calc Velocity: LV: %f, RV: %f", left_velocity, right_velocity);

        int left_speed = static_cast<int>(left_velocity * 120);
        int right_speed = static_cast<int>(right_velocity * 120);

        std::cout << "LEFT SPEED " << left_speed << std::endl;
        std::cout << "RIGHT SPEED " << right_speed << std::endl;

        // Control left motor
        if (left_speed >= 0)
        {
            gpio_write(pi_, L_IN1, 0);
            gpio_write(pi_, L_IN2, 1);
        }
        else
        {
            gpio_write(pi_, L_IN1, 1);
            gpio_write(pi_, L_IN2, 0);
        }
        set_PWM_dutycycle(pi_, L_PWM1, std::abs(left_speed));

        // Control right motor
        if (right_speed >= 0)
        {
            gpio_write(pi_, R_IN1, 0);
            gpio_write(pi_, R_IN2, 1);
        }
        else
        {
            gpio_write(pi_, R_IN1, 1);
            gpio_write(pi_, R_IN2, 0);
        }
        set_PWM_dutycycle(pi_, R_PWM1, std::abs(right_speed));
        // // Angular velocities in rad/s
        // float left_angular_velocity = static_cast<float>(hw_commands_[0]);
        // float right_angular_velocity = static_cast<float>(hw_commands_[1]);

        // // Log the received commands for debugging
        // RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Received commands: LF: %f, RF: %f", left_angular_velocity * 0.03, right_angular_velocity * 0.03);

        // // Assume max angular velocity corresponds to max PWM value (255 for 8-bit PWM)
        // const float max_angular_velocity = 16.67; // example value, adjust accordingly

        // // Scale angular velocities to PWM values
        // int left_pwm = static_cast<int>((left_angular_velocity / max_angular_velocity) * 100);
        // int right_pwm = static_cast<int>((right_angular_velocity / max_angular_velocity) * 100);

        // // Ensure PWM values are within the valid range (0-255)

        // std::cout << "LEFT PWM " << left_pwm << std::endl;
        // std::cout << "RIGHT PWM " << right_pwm << std::endl;

        // // Control left motor
        // if (left_angular_velocity >= 0)
        // {
        //     gpio_write(pi_, L_IN1, 0);
        //     gpio_write(pi_, L_IN2, 1);
        // } 
        // else 
        // {
        //     gpio_write(pi_, L_IN1, 1);
        //     gpio_write(pi_, L_IN2, 0);
        // }
        // set_PWM_dutycycle(pi_, L_PWM1, std::abs(left_pwm));

        //     // Control right motori
        // if (right_angular_velocity >= 0)
        // {
        //     gpio_write(pi_, R_IN1, 0);
        //     gpio_write(pi_, R_IN2, 1);
        // }
        // else
        // {
        //     gpio_write(pi_, R_IN1, 1);
        //     gpio_write(pi_, R_IN2, 0);
        // }
        // set_PWM_dutycycle(pi_, R_PWM1, std::abs(right_pwm));

        ///////////////////////////////////////////////////////////////////////////////

        // int left_speed = static_cast<int>((left_velocity / 0.628) * 255 * 0.85); // 0.628 = max_vel
        // int right_speed = static_cast<int>((right_velocity / 0.628) * 255);

        // left_speed = std::max(std::min(left_speed, 255), 0);
        // right_speed = std::max(std::min(right_speed, 255), 0);

        // if (left_velocity >= 0)
        // {i
        //     gpio_write(pi_, L_IN1, PI_HIGH);
        //     gpio_write(pi_, L_IN2, PI_LOW);
        // }
        // else
        // {
        //     gpio_write(pi_, L_IN1, PI_LOW);
        //     gpio_write(pi_, L_IN2, PI_HIGH);
        // }
        // set_PWM_dutycycle(pi_, L_PWM1, abs(255));

        // if (right_velocity >= 0)
        // {
        //     gpio_write(pi_, R_IN1, PI_HIGH);
        //     gpio_write(pi_, R_IN2, PI_LOW);

        //     gpio_write(pi_, 26, PI_LOW);
        //     gpio_write(pi_, 27, PI_HIGH);
        // }
        // else
        // {
        //     gpio_write(pi_, R_IN1, PI_LOW);
        //     gpio_write(pi_, R_IN2, PI_HIGH);

        //     gpio_write(pi_, 26, PI_HIGH);
        //     gpio_write(pi_, 27, PI_LOW);
        // }
        // set_PWM_dutycycle(pi_, R_PWM1, abs(right_speed));
        // set_PWM_dutycycle(pi_, 13, abs(right_speed));

        // Log the GPIO states for debugging
        // RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "GPIO states: L_IN1: %d, L_IN2: %d, L_PWM1: %d, R_IN1: %d, R_IN2: %d, R_PWM1: %d",
        //             gpio_read(pi_, L_IN1), gpio_read(pi_, L_IN2), gpio_read(pi_, L_PWM1),
        //             gpio_read(pi_, R_IN1), gpio_read(pi_, R_IN2), gpio_read(pi_, R_PWM1));

        // RCLCPP_INFO(
        //     rclcpp::get_logger("MyRobotHardware"),
        //     "Commands: LF: %f, RF: %f",
        //     left_velocity, right_velocity);

        return hardware_interface::return_type::OK;
    }

    void MyRobotHardware::cleanup()
    {
        gpio_write(pi_, L_IN1, 0);
        gpio_write(pi_, L_IN2, 0);
        gpio_write(pi_, L_PWM1, 0);

        gpio_write(pi_, R_IN1, 0);
        gpio_write(pi_, R_IN2, 0);
        gpio_write(pi_, R_PWM1, 0);

        pigpio_stop(pi_);

        RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "GPIO cleanup completed successfully.");
    }

    MyRobotHardware::~MyRobotHardware()
    {
        cleanup();
    }

} // namespace my_robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot::MyRobotHardware, hardware_interface::SystemInterface)