// Copyright (c) 2025 Elevate Robotics Inc
// Copyright (c) 2025 Synapticon GmbH
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "TechHardware.hpp"

#include <algorithm>
#include <chrono>
#include <sys/mman.h>
#include <sys/time.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <inttypes.h>
#include <rclcpp/rclcpp.hpp>

#define EC_TIMEOUTMON 500

using namespace std::chrono_literals;

namespace TechControlInterface
{
	namespace
	{
		constexpr char LOG_NAME[] = "TechControlInterface";
		constexpr double DEG_TO_RAD = 0.0174533;
		constexpr size_t PROFILE_TORQUE_MODE = 4;
		constexpr double RPM_TO_RAD_PER_S = 0.10472;
		constexpr double RAD_PER_S_TO_RPM = 1 / RPM_TO_RAD_PER_S;
		// Bit 2 (0-indexed) goes to 0 to turn on Quick Stop
		constexpr char EXPECTED_SLAVE_NAME[] = "? M:0000009a I:00030924";
		constexpr uint STACK128K{128 * 1024};
		constexpr uint NSEC_PER_SEC{1000000000};
	} // namespace

	int StepperPDOsetup(uint16 slave)
	{
		int retval = 0;
		uint8 zeroMap = 0;
		uint16 clearVal = 0x0000;
		uint32 mapObject;
		uint16 RPDOMap = 0x1c12;
		uint16 TPDOMap = 0x1c13;
		uint16 RPDOIdx = 0x1601;
		uint16 TPDOIdx = 0x1A00;
		uint16 RPDOMapIdx = 0x0001;
		uint16 TPDOMapIdx = 0x0001;
		uint8 mapCount = 4;
		//........................................................................................
		// Map RXPOD
		retval += ec_SDOwrite(slave, RPDOIdx, 0x00, FALSE, sizeof(zeroMap), &zeroMap, EC_TIMEOUTSAFE);

		// 2. Configure new PDO mapping
		// Control Word
		mapObject = 0x60400010; // 0x6040:0 Control Word, 16 bits
		retval += ec_SDOwrite(slave, RPDOIdx, 0x01, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Mode of Operation
		mapObject = 0x60600008; // 0x6060:0 Mode of Operation, 8 bits
		retval += ec_SDOwrite(slave, RPDOIdx, 0x02, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Target Position
		mapObject = 0x607A0020; // 0x607A:0 Target Position, 32 bits
		retval += ec_SDOwrite(slave, RPDOIdx, 0x03, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Profile Velocity
		mapObject = 0x60FF0020; // 0x60FF:0 Target Velocity, 32 bits
		retval += ec_SDOwrite(slave, RPDOIdx, 0x04, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Set number of mapped objects
		retval += ec_SDOwrite(slave, RPDOIdx, 0x00, FALSE, sizeof(mapCount), &mapCount, EC_TIMEOUTSAFE);

		// 4. Configure RXPDO allocation
		clearVal = 0x0000; // Clear the mapping
		retval += ec_SDOwrite(slave, RPDOMap, 0x00, FALSE, sizeof(clearVal), &clearVal, EC_TIMEOUTSAFE);
		retval += ec_SDOwrite(slave, RPDOMap, 0x01, FALSE, sizeof(RPDOIdx), &RPDOIdx, EC_TIMEOUTSAFE);
		retval += ec_SDOwrite(slave, RPDOMap, 0x00, FALSE, sizeof(RPDOMapIdx), &RPDOMapIdx, EC_TIMEOUTSAFE);

		//........................................................................................
		// Map TXPOD
		// First, clear the TXPDO mapping
		clearVal = 0x0000;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x00, FALSE, sizeof(clearVal), &clearVal, EC_TIMEOUTSAFE);

		// Configure TXPDO mapping entries
		// Status Word (0x6041:0, 16 bits)
		retval = 0;
		mapObject = 0x60410010;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x01, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Modes of Operation Display (0x6061:0, 8 bits)
		mapObject = 0x60610008;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x02, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Position Actual Value (0x6064:0, 32 bits)
		mapObject = 0x60640020;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x03, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Velocity Actual Value (0x60FD:0, 32 bits)
		mapObject = 0x606C0020;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x04, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Set the number of mapped objects (4 objects)
		mapCount = 4;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x00, FALSE, sizeof(mapCount), &mapCount, EC_TIMEOUTSAFE);

		// Configure TXPDO assignment
		// First, clear the assignment
		clearVal = 0x0000;
		retval += ec_SDOwrite(slave, TPDOMap, 0x00, FALSE, sizeof(clearVal), &clearVal, EC_TIMEOUTSAFE);
		retval += ec_SDOwrite(slave, TPDOMap, 0x01, FALSE, sizeof(TPDOIdx), &TPDOIdx, EC_TIMEOUTSAFE);
		retval += ec_SDOwrite(slave, TPDOMap, 0x00, FALSE, sizeof(TPDOMapIdx), &TPDOMapIdx, EC_TIMEOUTSAFE);

		while (EcatError)
			printf("%s", ec_elist2string());

		if (retval < 0)
		{
			printf("TXPDO Mapping failed\n");
			return (-1);
		}

		printf("slave %d set, retval = %d\n", slave, retval);
		return 1;
	}

	int TechPDOsetup(uint16 slave)
	{
		int retval = 0;
		uint8 zeroMap = 0;
		uint16 clearVal = 0x0000;
		uint32 mapObject;
		uint16 RPDOMap = 0x1c12;
		uint16 TPDOMap = 0x1c13;
		uint16 RPDOIdx = 0x1607;
		uint16 TPDOIdx = 0x1A07;
		uint16 RPDOMapIdx = 0x0001;
		uint16 TPDOMapIdx = 0x0001;
		uint8 mapCount = 4;
		//........................................................................................
		// Map RXPOD
		retval += ec_SDOwrite(slave, RPDOIdx, 0x00, FALSE, sizeof(zeroMap), &zeroMap, EC_TIMEOUTSAFE);

		// 2. Configure new PDO mapping
		// Control Word
		mapObject = 0x60400010; // 0x6040:0 Control Word, 16 bits
		retval += ec_SDOwrite(slave, RPDOIdx, 0x01, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Mode of Operation
		mapObject = 0x60600008; // 0x6060:0 Mode of Operation, 8 bits
		retval += ec_SDOwrite(slave, RPDOIdx, 0x02, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Target Position
		mapObject = 0x607A0020; // 0x607A:0 Target Position, 32 bits
		retval += ec_SDOwrite(slave, RPDOIdx, 0x03, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Profile Velocity
		mapObject = 0x60FF0020; // 0x60FF:0 Target Velocity, 32 bits
		retval += ec_SDOwrite(slave, RPDOIdx, 0x04, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Set number of mapped objects
		retval += ec_SDOwrite(slave, RPDOIdx, 0x00, FALSE, sizeof(mapCount), &mapCount, EC_TIMEOUTSAFE);

		// 4. Configure RXPDO allocation
		clearVal = 0x0000; // Clear the mapping
		retval += ec_SDOwrite(slave, RPDOMap, 0x00, FALSE, sizeof(clearVal), &clearVal, EC_TIMEOUTSAFE);
		retval += ec_SDOwrite(slave, RPDOMap, 0x01, FALSE, sizeof(RPDOIdx), &RPDOIdx, EC_TIMEOUTSAFE);
		retval += ec_SDOwrite(slave, RPDOMap, 0x00, FALSE, sizeof(RPDOMapIdx), &RPDOMapIdx, EC_TIMEOUTSAFE);

		//........................................................................................
		// Map TXPOD
		// First, clear the TXPDO mapping
		clearVal = 0x0000;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x00, FALSE, sizeof(clearVal), &clearVal, EC_TIMEOUTSAFE);

		// Configure TXPDO mapping entries
		// Status Word (0x6041:0, 16 bits)
		retval = 0;
		mapObject = 0x60410010;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x01, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Modes of Operation Display (0x6061:0, 8 bits)
		mapObject = 0x60610008;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x02, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Position Actual Value (0x6064:0, 32 bits)
		mapObject = 0x60640020;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x03, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Velocity Actual Value (0x60FD:0, 32 bits)
		mapObject = 0x606C0020;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x04, FALSE, sizeof(mapObject), &mapObject, EC_TIMEOUTSAFE);

		// Set the number of mapped objects (4 objects)
		mapCount = 4;
		retval += ec_SDOwrite(slave, TPDOIdx, 0x00, FALSE, sizeof(mapCount), &mapCount, EC_TIMEOUTSAFE);

		// Configure TXPDO assignment
		// First, clear the assignment
		clearVal = 0x0000;
		retval += ec_SDOwrite(slave, TPDOMap, 0x00, FALSE, sizeof(clearVal), &clearVal, EC_TIMEOUTSAFE);
		retval += ec_SDOwrite(slave, TPDOMap, 0x01, FALSE, sizeof(TPDOIdx), &TPDOIdx, EC_TIMEOUTSAFE);
		retval += ec_SDOwrite(slave, TPDOMap, 0x00, FALSE, sizeof(TPDOMapIdx), &TPDOMapIdx, EC_TIMEOUTSAFE);

		while (EcatError)
			printf("%s", ec_elist2string());

		if (retval < 0)
		{
			printf("TXPDO Mapping failed\n");
			return (-1);
		}

		printf("slave %d set, retval = %d\n", slave, retval);
		return 1;
	}

	hardware_interface::CallbackReturn TechHardwareInterface::on_init(
		const hardware_interface::HardwareInfo &info)
	{
		if (hardware_interface::SystemInterface::on_init(info) !=
			hardware_interface::CallbackReturn::SUCCESS)
		{
			return hardware_interface::CallbackReturn::ERROR;
		}
		m_Logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TechHardware"));
		m_NumJoints = info_.joints.size();

		for (const hardware_interface::ComponentInfo &joint : info_.joints)
		{
			if (!(joint.command_interfaces[0].name ==
					  hardware_interface::HW_IF_POSITION ||
				  joint.command_interfaces[0].name ==
					  hardware_interface::HW_IF_VELOCITY ||
				  joint.command_interfaces[0].name ==
					  "quick_stop" ||
				  joint.command_interfaces[0].name ==
					  hardware_interface::HW_IF_EFFORT))
			{
				RCLCPP_FATAL(
					get_logger(),
					"Joint '%s' has %s command interface. Expected %s, %s, %s, or %s.",
					joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
					hardware_interface::HW_IF_POSITION,
					hardware_interface::HW_IF_VELOCITY,
					"quick_stop",
					hardware_interface::HW_IF_EFFORT);
				return hardware_interface::CallbackReturn::ERROR;
			}

			if (!(joint.state_interfaces[0].name ==
					  hardware_interface::HW_IF_POSITION ||
				  joint.state_interfaces[0].name ==
					  hardware_interface::HW_IF_VELOCITY ||
				  joint.state_interfaces[0].name ==
					  hardware_interface::HW_IF_ACCELERATION ||
				  joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
			{
				RCLCPP_FATAL(
					get_logger(),
					"Joint '%s' has %s state interface. Expected %s, %s, %s, or %s.",
					joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
					hardware_interface::HW_IF_POSITION,
					hardware_interface::HW_IF_VELOCITY,
					hardware_interface::HW_IF_ACCELERATION,
					hardware_interface::HW_IF_EFFORT);
				return hardware_interface::CallbackReturn::ERROR;
			}
		}

		// Ethercat initialization
		// Define the interface name (e.g. eth0 or eno0) in the ros2_control.xacro
		std::string eth_device = info_.hardware_parameters["eth_device"];
		RCLCPP_INFO_STREAM(get_logger(), "Using eth_device: " << eth_device);
		if (ec_init(eth_device.c_str()) <= 0)
		{
			RCLCPP_FATAL_STREAM(get_logger(),
								"Error during initialization of ethercat interface: " << eth_device.c_str());
			return hardware_interface::CallbackReturn::ERROR;
		}

		if (ec_config_init(false) <= 0)
		{
			RCLCPP_FATAL(get_logger(), "No ethercat slaves found!");
			ec_close();
			return hardware_interface::CallbackReturn::ERROR;
		}

		for (int i = 1; i <= ec_slavecount; i++)
		{
			if (!strcmp(ec_slave[i].name, "ECT60V202"))
			{
				RCLCPP_INFO(get_logger(), "Binding slave %d with: ECT60V202", i); // Confirm successful state change
				ec_slave[i].PO2SOconfig = &StepperPDOsetup;
			}
			else
			{
				RCLCPP_INFO(get_logger(), "Binding slave %d with: Tech", i); // Confirm successful state change
				ec_slave[i].PO2SOconfig = &TechPDOsetup;
			}
		}

		ec_readstate(); // Read the state of the slaves

		for (int i = 1; i <= ec_slavecount; i++)
		{ // Loop through each slave
			if (ec_slave[i].state != EC_STATE_PRE_OP)
			{ // If the slave is not in PRE-OP state
				// Print the current state and status code of the slave
				ec_slave[i].state = EC_STATE_INIT; // Set the slave state to INIT
			}
			else
			{										 // If the slave is in PRE-OP state
				ec_slave[0].state = EC_STATE_PRE_OP; // Set the first slave to PRE-OP state
				/* Request EC_STATE_PRE_OP state for all slaves */
				ec_writestate(0); // Write the state change to the slave
				/* Wait for all slaves to reach the PRE-OP state */
				if ((ec_statecheck(0, EC_STATE_PRE_OP, 3 * EC_TIMEOUTSTATE)) == EC_STATE_PRE_OP)
				{
					RCLCPP_INFO(get_logger(), "Successfully changed to PRE_OP state"); // Confirm successful state change
				}
				else
				{
					RCLCPP_FATAL(get_logger(), "State EC_STATE_PRE_OP cannot be changed");
					return hardware_interface::CallbackReturn::ERROR;
				}
			}
		}

		ecx_context.manualstatechange = 1; // Disable automatic state change
		osal_usleep(1e6);				   // Sleep for 1 second

		// Print the information of the slaves found
		for (int i = 1; i <= ec_slavecount; i++)
		{
			// (void)ecx_FPWR(ecx_context.port, i, ECT_REG_DCSYNCACT, sizeof(WA), &WA, 5 * EC_TIMEOUTRET);
			RCLCPP_INFO(get_logger(), "Name: %s\t", ec_slave[i].name); // Print the name of the slave
			RCLCPP_INFO(get_logger(), "Slave %d: Type %d, Address 0x%02x, State Machine actual %d, required %d",
						i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_INIT);
			ecx_dcsync0(&ecx_context, i, TRUE, m_CycleTime * 1000, 0); // Synchronize the distributed clock for the slave
		}

		// Map the configured PDOs to the IOmap
		ec_config_map(&m_IOMap);

		for (int i = 1; i <= ec_slavecount; i++)
		{
			if (ec_slave[i].state != EC_STATE_PRE_OP)
			{ // Check if the slave is not in PRE-OP state
				RCLCPP_FATAL(get_logger(), "Slave %d not in PRE-OP state. Current state: %d\n", i, ec_slave[i].state);
				return hardware_interface::CallbackReturn::ERROR;
			}
		}

		ec_configdc();

		// Request to switch to SAFE-OP state
		ec_slave[0].state = EC_STATE_SAFE_OP; // Set the first slave to SAFE-OP state
		ec_writestate(0);					  // Write the state change to the slave

		// Wait for the state transition
		if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP)
		{
			RCLCPP_INFO(get_logger(), "Successfully changed to SAFE_OP state"); // Confirm successful state change
		}
		else
		{
			RCLCPP_INFO(get_logger(), "Failed to change to SAFE_OP state");
			return hardware_interface::CallbackReturn::ERROR;
		}

		// Request operational state for all slaves
		m_ExpectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
		RCLCPP_INFO(get_logger(), "ExpectedWKC: %d", (int)m_ExpectedWKC);
		// Read and display basic status information of the slaves
		ec_readstate(); // Read the state of all slaves
		for (int i = 1; i <= ec_slavecount; i++)
		{
			RCLCPP_INFO(get_logger(), "Slave %d\t", i);
			RCLCPP_INFO(get_logger(), "  State: %02x\t", ec_slave[i].state);			   // Print the state of the slave
			RCLCPP_INFO(get_logger(), "  ALStatusCode: %04x\t", ec_slave[i].ALstatuscode); // Print the AL status code
			RCLCPP_INFO(get_logger(), "  Delay: %d\t", ec_slave[i].pdelay);				   // Print the delay of the slave
			RCLCPP_INFO(get_logger(), "  Has DC: %d\t", ec_slave[i].hasdc);				   // Check if the slave supports Distributed Clock
			RCLCPP_INFO(get_logger(), "  DC Active: %d\t", ec_slave[i].DCactive);		   // Check if DC is active for the slave
			RCLCPP_INFO(get_logger(), "  DC supported: %d\t", ec_slave[i].hasdc);		   // Print if DC is supported
		}

		// Read DC synchronization configuration using the correct parameters
		for (int i = 1; i <= ec_slavecount; i++)
		{
			uint16_t dcControl = 0; // Variable to hold DC control configuration
			int32_t cycleTime = 0;	// Variable to hold cycle time
			int size;				// Variable to hold size for reading

			// Read DC synchronization configuration, adding the correct size parameter
			size = sizeof(dcControl);
			if (ec_SDOread(i, 0x1C32, 0x01, FALSE, &size, &dcControl, EC_TIMEOUTSAFE) > 0)
			{
				RCLCPP_INFO(get_logger(), "Slave %d DC Configuration:", i);
				RCLCPP_INFO(get_logger(), "  DC Control: 0x%04x", dcControl); // Print the DC control configuration

				size = sizeof(cycleTime);
				if (ec_SDOread(i, 0x1C32, 0x02, FALSE, &size, &cycleTime, EC_TIMEOUTSAFE) > 0)
				{
					RCLCPP_INFO(get_logger(), "  Cycle Time: %d ns", cycleTime); // Print the cycle time
				}
			}
		}
		m_EcatErrorThread = std::thread(&TechHardwareInterface::ecatCheck, this, (void *)&ctime);
		m_EcatErrorThread->detach();

		// Start the control loop, wait for it to reach normal operation mode
		m_SomanetControlThread = std::thread(&TechHardwareInterface::somanetCyclicLoop, this, std::ref(m_InNomalOPMode));
		m_SomanetControlThread->detach();
		// request OP state for all slaves
		ec_slave[0].state = EC_STATE_OPERATIONAL; // Change the state of the first slave to OP
		ec_writestate(0);
		size_t chk = 200;
		do
		{
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);
			ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
		} while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
		m_InNomalOPMode = true;

		if (ec_slave[0].state != EC_STATE_OPERATIONAL)
		{
			RCLCPP_FATAL(get_logger(), "An ethercat slave failed to reach OPERATIONAL state");
			return hardware_interface::CallbackReturn::ERROR;
		}

		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn TechHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
	{
		return CallbackReturn::SUCCESS;
	}

	hardware_interface::return_type TechHardwareInterface::prepare_command_mode_switch(
		const std::vector<std::string> &start_interfaces,
		const std::vector<std::string> &stop_interfaces)
	{
		// Prepare for new command modes
		std::vector<ControlLevelEnum> newModes = {};

		if (start_interfaces.empty())
		{
			RCLCPP_FATAL(get_logger(), "Empty activate interfaces is not allowed!");
			return hardware_interface::return_type::ERROR;
		}
		else
		{
			// All joints must have the same command mode
			if (!std::all_of(newModes.begin() + 1, newModes.end(), [&](ControlLevelEnum mode)
							 { return mode == newModes[0]; }))
			{
				RCLCPP_FATAL(get_logger(), "All joints must have the same command mode.");
				return hardware_interface::return_type::ERROR;
			}
			for (const std::string &key : start_interfaces)
			{
				for (std::size_t i = 0; i < m_NumJoints; i++)
				{
					if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
					{
						newModes.push_back(ControlLevelEnum::VELOCITY);
					}
					else if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
					{
						newModes.push_back(ControlLevelEnum::POSITION);
					}
					else if (key == info_.joints[i].name + "/" + "disable")
					{
						newModes.push_back(ControlLevelEnum::DISABLE);
					}
				}
			}
		}

		resetData();

		for (const std::string &key : start_interfaces)
		{
			for (std::size_t i = 0; i < m_NumJoints; i++)
			{
				if (key.find(info_.joints[i].name) != std::string::npos)
				{
					m_ControlLevel[i] = newModes[i];
				}
			}
		}

		return hardware_interface::return_type::OK;
	}

	hardware_interface::CallbackReturn TechHardwareInterface::on_activate(
		const rclcpp_lifecycle::State & /*previous_state*/)
	{

		// Set some default values
		for (std::size_t i = 0; i < m_NumJoints; i++)
		{
		}

		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn TechHardwareInterface::on_deactivate(
		const rclcpp_lifecycle::State & /*previous_state*/)
	{

		for (std::size_t i = 0; i < m_NumJoints; i++)
		{
			m_ControlLevel[i] = ControlLevelEnum::DISABLE;
			m_InterfacePositionStates[i] = std::numeric_limits<double>::quiet_NaN();
			m_InterfaceVelocityStates[i] = 0;
			m_TargetPosition[i] = std::numeric_limits<double>::quiet_NaN();
			m_TargetVelocity[i] = 0;
		}

		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::return_type
	TechHardwareInterface::read(const rclcpp::Time & /*time*/,
								const rclcpp::Duration & /*period*/)
	{
		for (std::size_t i = 0; i < m_NumJoints; i++)
		{
			m_InterfacePositionStates[i] = m_InTechDrive[i]->PositionActualValue;
			m_InterfaceVelocityStates[i] = m_InTechDrive[i]->VelocityActualValue;
			// RCLCPP_INFO(get_logger(), "setting %s Pos: %f, Vel: %f", std::string(info_.joints[i].name).c_str(), m_InterfacePositionStates[i], m_InterfaceVelocityStates[i]);
		}

		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type
	TechHardwareInterface::write(const rclcpp::Time & /*time*/,
								 const rclcpp::Duration & /*period*/)
	{
		for (std::size_t i = 0; i < m_NumJoints; i++)
		{
			// RCLCPP_INFO(get_logger(), "%s Get command: Pos: %f\tVel: %f\t", std::string(info_.joints[i].name).c_str(), m_InterfacePositionCommands[i], m_InterfaceVelocityCommands[i]);
			// Thread of this function is seperated form loop. The command really write into motor drive in loop.
			if (!std::isnan(m_InterfacePositionCommands[i]))
			{
				m_TargetPosition[i] = m_InterfacePositionCommands[i];
			}
			m_TargetVelocity[i] = m_InterfaceVelocityCommands[i];
		}

		return hardware_interface::return_type::OK;
	}

	std::vector<hardware_interface::StateInterface> TechHardwareInterface::export_state_interfaces()
	{
		std::vector<hardware_interface::StateInterface> state_interfaces;
		for (std::size_t i = 0; i < m_NumJoints; ++i)
		{
			state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_InterfacePositionStates[i]));
			state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &m_InterfaceVelocityStates[i]));
		}
		return state_interfaces;
	}

	std::vector<hardware_interface::CommandInterface> TechHardwareInterface::export_command_interfaces()
	{
		std::vector<hardware_interface::CommandInterface> command_interfaces;
		for (std::size_t i = 0; i < m_NumJoints; ++i)
		{
			command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_InterfacePositionCommands[i]));
			command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &m_InterfaceVelocityCommands[i]));
			command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "disable", &m_InterfaceDisableCommands[i]));
		}
		return command_interfaces;
	}

	TechHardwareInterface::~TechHardwareInterface()
	{
		// A flag to ecat_error_check_ thread
		m_InNomalOPMode = false;
		// if (m_SomanetControlThread && m_SomanetControlThread->joinable())
		// {
		// 	m_SomanetControlThread->join();
		// }
		while (1)
		{
            osal_usleep(100000); // Sleep for 100ms to reduce CPU usage
		}
		// Close the ethercat connection
		ec_close();
	}

	OSAL_THREAD_FUNC TechHardwareInterface::ecatCheck(void * /*ptr*/)
	{
		int slave;
		uint8 currentgroup = 0;

		while (1)
		{

			if (m_InNomalOPMode && ((m_WKC < m_ExpectedWKC) || ec_group[currentgroup].docheckstate))
			{
				// one or more slaves are not responding
				ec_group[currentgroup].docheckstate = false;
				ec_readstate();
				for (slave = 1; slave <= ec_slavecount; slave++)
				{
					if (ec_slave[slave].name != EXPECTED_SLAVE_NAME)
					{
						continue;
					}
					if ((ec_slave[slave].group == currentgroup) &&
						(ec_slave[slave].state != EC_STATE_OPERATIONAL))
					{
						ec_group[currentgroup].docheckstate = true;
						if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
						{
							RCLCPP_WARN(get_logger(), "ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.", slave);
							ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
							ec_writestate(slave);
						}
						else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
						{
							RCLCPP_WARN(get_logger(), "WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.", slave);
							ec_slave[slave].state = EC_STATE_OPERATIONAL;
							ec_writestate(slave);
						}
						else if (ec_slave[slave].state > EC_STATE_NONE)
						{
							if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
							{
								ec_slave[slave].islost = false;
								RCLCPP_WARN(get_logger(), "MESSAGE : slave %d reconfigured", slave);
							}
						}
						else if (!ec_slave[slave].islost)
						{
							// re-check state
							ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
							if (ec_slave[slave].state == EC_STATE_NONE)
							{
								ec_slave[slave].islost = true;
								RCLCPP_WARN(get_logger(), "ERROR : slave %d lost", slave);
							}
						}
					}
					if (ec_slave[slave].islost)
					{
						if (ec_slave[slave].state == EC_STATE_NONE)
						{
							if (ec_recover_slave(slave, EC_TIMEOUTMON))
							{
								ec_slave[slave].islost = false;
								RCLCPP_INFO(get_logger(), "MESSAGE : slave %d recovered", slave);
							}
						}
						else
						{
							ec_slave[slave].islost = false;
							RCLCPP_INFO(get_logger(), "MESSAGE : slave %d found", slave);
						}
					}
				}
				if (!ec_group[currentgroup].docheckstate)
					RCLCPP_INFO(get_logger(), "OK : all slaves resumed OPERATIONAL.");
			}
			osal_usleep(10000);
		}
	}

	void TechHardwareInterface::somanetCyclicLoop(std::atomic<bool> &inNormalOPMode)
	{
		RCLCPP_INFO(get_logger(), "somanetCyclicLoop is launched!");
		timespec ts, tleft;
		int missedCycles = 0;
		const int MAX_MISSED_CYCLES = 10;
		timespec cycleStart, cycleEnd;
		long cycleTimeNanoSec;

		clock_gettime(CLOCK_MONOTONIC, &ts);
		int ht = (ts.tv_nsec / 1000000) + 1;
		ts.tv_nsec = ht * 1000000;
		if (ts.tv_nsec >= NSEC_PER_SEC)
		{
			ts.tv_sec++;
			ts.tv_nsec -= NSEC_PER_SEC;
		}
		int64 cycletime = m_CycleTime * 1000;

		m_TOff = 0;

		// Connect struct pointers to I/O
		for (size_t i = 1; i < (m_NumJoints + 1); ++i)
		{
			m_InTechDrive.push_back((InTechDrive *)ec_slave[i].inputs);
			m_OutTechDrive.push_back((OutTechDrive *)ec_slave[i].outputs);
		}
		resetData();

		std::vector<int> repeats(ec_slavecount, 0);
		std::vector<OutTechDrive> repeatData(ec_slavecount);
		while (rclcpp::ok())
		{
			std::lock_guard<std::mutex> lock(m_InSomanetMtx);
			clock_gettime(CLOCK_MONOTONIC, &cycleStart);

			AddTimespec(&ts, cycletime + m_TOff);
			if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft) != 0)
			{
				// If sleep is interrupted, record the error
				missedCycles++;
				RCLCPP_WARN(get_logger(), "Clock sleep interrupted, missed cycles: %d", missedCycles);
				if (missedCycles >= MAX_MISSED_CYCLES)
				{
					RCLCPP_FATAL(get_logger(), "Too many missed cycles, attempting recovery...");
					// Reset the counter
					missedCycles = 0;
					// Resynchronize the clock
					clock_gettime(CLOCK_MONOTONIC, &ts);
					ts.tv_nsec = ((ts.tv_nsec / 1000000) + 1) * 1000000;
					if (ts.tv_nsec >= NSEC_PER_SEC)
					{
						ts.tv_sec++;
						ts.tv_nsec -= NSEC_PER_SEC;
					}
				}
			}
			else
			{
				missedCycles = 0;
			}

			m_WKC = ec_receive_processdata(EC_TIMEOUTRET);

			if (inNormalOPMode)
			{
				if (m_WKC >= m_ExpectedWKC)
				{

					for (int i = 1; i <= ec_slavecount; ++i)
					{
						if (repeats[i - 1] != 0)
						{
							*m_OutTechDrive[i - 1] = repeatData[i - 1];
							--repeats[i - 1];
						}
						else if ((m_InTechDrive[i - 1]->StatusWord & 0x0008) == 0x0008)
						{
							m_OutTechDrive[i - 1]->ControlWord = 0x80;
							repeatData[i - 1] = *m_OutTechDrive[i - 1];
							repeats[i - 1] = 200;
						}
						else if ((m_InTechDrive[i - 1]->StatusWord & 0x0040) == 0x0040)
						{
							m_OutTechDrive[i - 1]->ControlWord = 0x6;
							repeatData[i - 1] = *m_OutTechDrive[i - 1];
							repeats[i - 1] = 200;
						}
						else if ((m_InTechDrive[i - 1]->StatusWord & 0x003F) == 0x0031)
						{
							m_OutTechDrive[i - 1]->ControlWord = 0x7;
							repeatData[i - 1] = *m_OutTechDrive[i - 1];
							repeats[i - 1] = 200;
						}
						else if (m_ControlLevel[i - 1] != ControlLevelEnum::DISABLE && (m_InTechDrive[i - 1]->StatusWord & 0x003F) == 0x0033)
						{
							m_OutTechDrive[i - 1]->ControlWord = 0xF;
							m_OutTechDrive[i - 1]->TargetVelocity = 0;
							m_OutTechDrive[i - 1]->TargetPosition = m_InTechDrive[i - 1]->PositionActualValue;
							repeatData[i - 1] = *m_OutTechDrive[i - 1];
							repeats[i - 1] = 200;
						}
						else if ((m_InTechDrive[i - 1]->StatusWord & 0x003F) == 0x0037)
						{
							if (m_ControlLevel[i - 1] == ControlLevelEnum::POSITION)
							{
								m_OutTechDrive[i - 1]->TargetVelocity = 0;
								if (!std::isnan(m_TargetPosition[i - 1]))
								{
									m_OutTechDrive[i - 1]->OpMode = 8;
									m_OutTechDrive[i - 1]->TargetPosition = m_TargetPosition[i - 1];
									m_OutTechDrive[i - 1]->ControlWord = 0x0F;
								}
							}
							else if (m_ControlLevel[i - 1] == ControlLevelEnum::VELOCITY)
							{
								m_OutTechDrive[i - 1]->OpMode = 9;
								m_OutTechDrive[i - 1]->TargetVelocity = m_TargetVelocity[i - 1];
								m_OutTechDrive[i - 1]->ControlWord = 0x0F;
							}
							else if (m_ControlLevel[i - 1] == ControlLevelEnum::DISABLE)
							{
								m_OutTechDrive[i - 1]->TargetVelocity = 0;
								m_OutTechDrive[i - 1]->ControlWord = 0b1011;
							}
						}
					}
				}
				else
				{
					RCLCPP_WARN(get_logger(), "Working counter error (wkc: %d, expected: %d)\n", (int)m_WKC, (int)m_ExpectedWKC);
				}
			}

			// Clock synchronization
			if (ec_slave[0].hasdc)
			{
				ECSync(ec_DCtime, cycletime, &m_TOff);
			}

			// Send process data
			ec_send_processdata();
			clock_gettime(CLOCK_MONOTONIC, &cycleEnd);
			cycleTimeNanoSec = (cycleEnd.tv_sec - cycleStart.tv_sec) * NSEC_PER_SEC +
							   (cycleEnd.tv_nsec - cycleStart.tv_nsec);

			if (cycleTimeNanoSec > cycletime * 1.5)
			{
				RCLCPP_WARN(get_logger(), "Cycle time exceeded: %ld ns (expected: %ld ns)", cycleTimeNanoSec, cycletime);
			}
		}
	}

	void TechHardwareInterface::resetData()
	{
		m_TargetPosition.resize(m_NumJoints);
		for (auto &val : m_TargetPosition)
		{
			val.store(std::numeric_limits<double>::quiet_NaN());
		}
		m_TargetVelocity.resize(m_NumJoints);
		for (auto &val : m_TargetVelocity)
		{
			val.store(0);
		}
		m_ControlLevel.resize(m_NumJoints);
		for (auto &val : m_ControlLevel)
		{
			val.store(ControlLevelEnum::DISABLE);
		}
		m_InterfacePositionCommands.clear();
		m_InterfacePositionStates.clear();
		m_InterfaceVelocityCommands.clear();
		m_InterfaceVelocityStates.clear();
		m_InterfaceDisableCommands.clear();
		m_InterfacePositionCommands.resize(m_NumJoints, std::numeric_limits<double>::quiet_NaN());
		m_InterfacePositionStates.resize(m_NumJoints, std::numeric_limits<double>::quiet_NaN());
		m_InterfaceVelocityCommands.resize(m_NumJoints, 0);
		m_InterfaceVelocityStates.resize(m_NumJoints, 0);
		m_InterfaceDisableCommands.resize(m_NumJoints, 0);
	}

	void TechHardwareInterface::AddTimespec(timespec *ts, int64 addtime)
	{
		int64 sec, nsec; // Variables to hold seconds and nanoseconds

		nsec = addtime % NSEC_PER_SEC;		   // Calculate nanoseconds to add
		sec = (addtime - nsec) / NSEC_PER_SEC; // Calculate seconds to add
		ts->tv_sec += sec;					   // Update seconds in timespec
		ts->tv_nsec += nsec;				   // Update nanoseconds in timespec
		if (ts->tv_nsec >= NSEC_PER_SEC)
		{													   // If nanoseconds exceed 1 second
			nsec = ts->tv_nsec % NSEC_PER_SEC;				   // Adjust nanoseconds
			ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC; // Increment seconds
			ts->tv_nsec = nsec;								   // Set adjusted nanoseconds
		}
	}

	void TechHardwareInterface::ECSync(int64 reftime, int64 cycletime, int64 *offsettime)
	{
		static int64 integral = 0;	   // Integral term for PI TechController
		int64 delta;				   // Variable to hold the difference between reference time and cycle time
		delta = (reftime) % cycletime; // Calculate the delta time
		if (delta > (cycletime / 2))
		{
			delta = delta - cycletime; // Adjust delta if it's greater than half the cycle time
		}
		if (delta > 0)
		{
			integral++; // Increment integral if delta is positive
		}
		if (delta < 0)
		{
			integral--; // Decrement integral if delta is negative
		}
		*offsettime = -(delta / 100) - (integral / 20); // Calculate the offset time
		m_GlobalDelta = delta;							// Update global delta variable
	}

} // namespace TechControlInterface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(TechControlInterface::TechHardwareInterface,
					   hardware_interface::SystemInterface)
