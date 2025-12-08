#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <pigpiod_if2.h>

// ----------------------------------------------------
// STEPPER CONTROLLER
// ----------------------------------------------------
class StepperPWM {
private:
    int pi;
    int pulsePin;
    int dirPin;
    int enPin;

public:
    StepperPWM(int pi_handle, int pulse, int dir, int en)
        : pi(pi_handle), pulsePin(pulse), dirPin(dir), enPin(en)
    {
        set_mode(pi, pulsePin, PI_OUTPUT);
        set_mode(pi, dirPin, PI_OUTPUT);
        set_mode(pi, enPin, PI_OUTPUT);

        gpio_write(pi, dirPin, 1);
        gpio_write(pi, enPin, 0);
    }

    void enable(bool on) {
        gpio_write(pi, enPin, on ? 0 : 1);
    }

    void setDirection(bool cw) {
        gpio_write(pi, dirPin, cw ? 1 : 0);
    }

    void setSpeed(unsigned freq) {
        if (freq == 0) {
            stop();
            return;
        }
        set_PWM_frequency(pi, pulsePin, freq);
        set_PWM_dutycycle(pi, pulsePin, 128);
    }

    void stop() {
        set_PWM_dutycycle(pi, pulsePin, 0);
    }
};

// ----------------------------------------------------
// BLDC SERVO CONTROLLER
// ----------------------------------------------------
class BLDCMotor {
private:
    int pi;
    int pwmPin;

public:
    BLDCMotor(int pi_handle, int pin)
        : pi(pi_handle), pwmPin(pin)
    {
        set_mode(pi, pwmPin, PI_OUTPUT);
    }

    void setSpeed(int pulse_us) {
        if (pulse_us < 1000) pulse_us = 1000;
        if (pulse_us > 2000) pulse_us = 2000;

        set_servo_pulsewidth(pi, pwmPin, pulse_us);
    }

    void stop() {
        set_servo_pulsewidth(pi, pwmPin, 1500);
    }
};

// ----------------------------------------------------
// MAIN PROGRAM
// ----------------------------------------------------
int main() {
    // Start pigpiod if needed
    if (system("pgrep pigpiod > /dev/null") != 0) {
        std::cout << "Starting pigpiod...\n";
        system("sudo pigpiod");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cerr << "Failed to connect to pigpio daemon!\n";
        return 1;
    }

    // Create both controllers
    StepperPWM stepper(pi, 17, 27, 22);
    BLDCMotor bldc(pi, 18);

    int mode = 0;  // 0=idle, 1=stepper, 2=bldc, 3=both

    std::cout << "\n=== Dual Motor Control ===\n";
    std::cout << "Commands:\n";
    std::cout << "  m - select mode (0=off,1=stepper,2=bldc,3=both)\n";
    std::cout << "  e - enable motors\n";
    std::cout << "  d - set stepper direction\n";
    std::cout << "  s - set speed (dependent on mode)\n";
    std::cout << "  x - stop motors\n";
    std::cout << "  q - quit\n";

    while (true) {
        std::cout << "\n> ";
        char cmd;
        std::cin >> cmd;

        if (cmd == 'm') {
            std::cout << "Mode? (0 off, 1 stepper, 2 bldc, 3 both): ";
            std::cin >> mode;
            std::cout << "Mode set to " << mode << "\n";
        }

        else if (cmd == 'e') {
            if (mode == 1 || mode == 3) stepper.enable(true);
            if (mode == 2 || mode == 3) bldc.setSpeed(1500);  // safe neutral
            std::cout << "Motors enabled.\n";
        }

        else if (cmd == 'd') {
            if (mode == 1 || mode == 3) {
                std::cout << "Direction? (0=CCW,1=CW): ";
                int dir;
                std::cin >> dir;
                stepper.setDirection(dir != 0);
            } else {
                std::cout << "Stepper disabled in this mode.\n";
            }
        }

        else if (cmd == 's') {
            if (mode == 1 || mode == 3) {
                std::cout << "Stepper speed (Hz): ";
                unsigned f;
                std::cin >> f;
                stepper.setSpeed(f);
            }

            if (mode == 2 || mode == 3) {
                std::cout << "BLDC speed (1000=rev,1500=stop,2000=fwd): ";
                int pw;
                std::cin >> pw;
                bldc.setSpeed(pw);
            }
        }

        else if (cmd == 'x') {
            stepper.stop();
            bldc.stop();
            std::cout << "Motors stopped.\n";
        }

        else if (cmd == 'q') {
            stepper.stop();
            bldc.stop();
            pigpio_stop(pi);
            return 0;
        }

        else {
            std::cout << "Unknown command.\n";
        }
    }

    return 0;
}
