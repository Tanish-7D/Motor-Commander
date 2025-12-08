#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <pigpiod_if2.h>
#include <rclcpp/rclcpp.hpp>
#include "motor_control/msg/flow_data.hpp"

// Simple PWM stepper controller using pigpio
class StepperPWM {
private:
    int pi_;
    int pulsePin_;
    int dirPin_;
    int enPin_;

public:
    StepperPWM(int pulse, int dir, int en)
        : pulsePin_(pulse), dirPin_(dir), enPin_(en) 
    {
        // Start pigpiod if not running
        if (system("pgrep pigpiod > /dev/null") != 0) {
            std::cout << "Starting pigpiod..." << std::endl;
            system("sudo pigpiod");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        pi_ = pigpio_start(NULL, NULL);
        if (pi_ < 0) {
            std::cerr << "Failed to connect to pigpio daemon!" << std::endl;
            exit(1);
        }

        set_mode(pi_, pulsePin_, PI_OUTPUT);
        set_mode(pi_, dirPin_, PI_OUTPUT);
        set_mode(pi_, enPin_, PI_OUTPUT);

        gpio_write(pi_, dirPin_, 1);  // default direction
        gpio_write(pi_, enPin_, 0);   // enable active low
    }

    ~StepperPWM() {
        stop();
        enable(false);
        pigpio_stop(pi_);
    }

    void enable(bool on) {
        gpio_write(pi_, enPin_, on ? 0 : 1);
    }

    void setDirection(bool clockwise) {
        gpio_write(pi_, dirPin_, clockwise ? 1 : 0);
    }

    void setSpeed(unsigned frequency) {
        // Debug print
        std::cout << "[DEBUG] setSpeed called with frequency: " << frequency << " Hz" << std::endl;

        if (frequency == 0) {
            stop();
            return;
        }

        int actual = set_PWM_frequency(pi_, pulsePin_, frequency);
        set_PWM_dutycycle(pi_, pulsePin_, 128); // 50% duty cycle
        std::cout << "[DEBUG] Requested freq: " << frequency 
                  << " Hz, pigpio set_PWM_frequency returned: " << actual << std::endl;
    }

    void stop() {
        set_PWM_dutycycle(pi_, pulsePin_, 0);
    }
};

// ROS2 Node controlling the stepper motor
class StepperNode : public rclcpp::Node {
private:
    StepperPWM motor_;
    rclcpp::Subscription<motor_control::msg::FlowData>::SharedPtr flow_sub_;
    const unsigned max_frequency_ = 1000;   // Max PWM frequency
    const unsigned min_frequency_ = 50;     // Min safe frequency
    const unsigned default_frequency_ = 500; // Initial frequency

public:
    StepperNode() : Node("stepper_pwm"), motor_(17, 27, 22) {
        motor_.enable(true);
        motor_.setSpeed(default_frequency_);
        RCLCPP_INFO(this->get_logger(), "Motor started at default frequency: %u Hz", default_frequency_);

        // Define QoS for cross-distro compatibility
        rclcpp::QoS qos(rclcpp::KeepLast(50));
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);

        // Subscribe to FlowData topic
        flow_sub_ = this->create_subscription<motor_control::msg::FlowData>(
            "/extrusion_flow_rate",
            qos,
            [this](motor_control::msg::FlowData::SharedPtr msg) {

                // Debug print: message received
                RCLCPP_INFO(this->get_logger(),
                    "[DEBUG] FlowData received: stepper_current_flowrate=%f, stepper_optimal_flowrate=%f",
                    msg->stepper_current_flowrate,
                    msg->stepper_optimal_flowrate);

                if (msg->stepper_optimal_flowrate <= 0) {
                    RCLCPP_WARN(this->get_logger(), "Invalid optimal flowrate: %f", msg->stepper_optimal_flowrate);
                    return;
                }

                double proportion = msg->stepper_current_flowrate / msg->stepper_optimal_flowrate;
                unsigned freq = static_cast<unsigned>(max_frequency_ * proportion);

                // Clamp frequency
                if (freq < min_frequency_) freq = min_frequency_;
                if (freq > max_frequency_) freq = max_frequency_;

                // Debug print: frequency calculated
                RCLCPP_INFO(this->get_logger(),
                    "[DEBUG] Calculated PWM frequency: %u Hz (proportion=%.3f)", freq, proportion);

                motor_.setSpeed(freq);
                RCLCPP_INFO(this->get_logger(), "PWM frequency updated: %u Hz", freq);
            }
        );
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StepperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
