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

#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "ethercat.h"

namespace TechControlInterface
{

    #pragma pack(1)
    // Structure for RXPDO (Control data sent to slave)
    typedef struct {
        uint16 ControlWord;
        int8 OpMode;
        int32 TargetPosition;
        int32 TargetVelocity;
    } OutTechDrive;
    
    // Structure for TXPDO (Status data received from slave)
    typedef struct {
        uint16 StatusWord;
        int8 OpModeDisplay;
        int32 PositionActualValue;
        int32 VelocityActualValue;

    } InTechDrive;
    #pragma pack()

    class TechHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(TechHardwareInterface)

        ~TechHardwareInterface();

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;


        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
      
        /**
         * \return logger of the SystemInterface.
         */
        rclcpp::Logger get_logger() const { return *m_Logger; }

    private:
        void resetData();
        void AddTimespec(struct timespec *ts, int64 addtime);
        void ECSync(int64 reftime, int64 cycletime, int64 *offsettime);

    private:
        /**
         * @brief Error checking. Typically runs in a separate thread.
         */
        OSAL_THREAD_FUNC ecatCheck(void *ptr);

        /**
         * @brief Somanet control loop runs in a dedicated thread
         * This steps through several states to get to Operational, if needed
         * @param inNomalOPMode A flag to the main thread that the Somanet state
         * machine is ready
         */
        void somanetCyclicLoop(std::atomic<bool> &inNomalOPMode);

        std::optional<std::thread> m_SomanetControlThread;

        size_t m_NumJoints;

        // Objects for logging
        std::shared_ptr<rclcpp::Logger> m_Logger;


        // Enum defining current control level
        enum class ControlLevelEnum : std::uint8_t
        {
            UNDEFINED = 0,
            EFFORT = 1, // aka torque
            VELOCITY = 2,
            POSITION = 3,
            QUICK_STOP = 4,
        };
        std::vector<double> m_InterfacePositionCommands;
        std::vector<double> m_InterfacePositionStates;
        std::vector<double> m_InterfaceVelocityCommands;
        std::vector<double> m_InterfaceVelocityStates;
        std::deque<std::atomic<double>> m_TargetPosition;
        std::deque<std::atomic<double>> m_TargetVelocity;
        std::deque<std::atomic<ControlLevelEnum>> m_ControlLevel;

        // For SOEM
        OSAL_THREAD_HANDLE m_EcatErrorThread;
        char m_IOMap[4096];
        int m_CycleTime{5000};
        int64 m_TOff;
        int64 m_GlobalDelta;

        std::vector<InTechDrive*> m_InTechDrive;
        std::mutex m_InSomanetMtx;
        std::vector<OutTechDrive*> m_OutTechDrive;


        // For coordination between threads
        volatile std::atomic<int> m_WKC;
        std::atomic<int> m_ExpectedWKC;
        std::atomic<bool> m_NeedLF = false;
        std::atomic<bool> m_InNomalOPMode = false;
    };

} // namespace TechControlInterface