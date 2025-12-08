#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <pigpiod_if2.h>


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
    //StepperPWM stepper(pi, 17, 27, 22);
    BLDCMotor bldc(pi, 18);

    int mode = 0;  // 0=idle, 1=stepper, 2=bldc, 3=both

    std::cout << "\n=== Motor Control ===\n";
    std::cout << "Commands:\n";
    std::cout << "  m - select mode (0=off,1=bldc)\n";
    std::cout << "  e - enable motor\n";
    //std::cout << "  d - set stepper direction\n";
    std::cout << "  s - set speed (dependent on mode)\n";
    std::cout << "  x - stop motors\n";
    std::cout << "  q - quit\n";

    while (true) {
        std::cout << "\n> ";
        char cmd;
        std::cin >> cmd;
        if (cmd == 'm') {
            std::cout << "Mode? (0 off, 1 bldc): ";
            std::cin >> mode;
            std::cout << "Mode set to " << mode << "\n";
        }

        else if (cmd == 'e') {
            //if (mode == 1 ) stepper.enable(true);
            if (mode == 1) bldc.setSpeed(1500);  // safe neutral
            std::cout << "Motor enabled.\n";
        }

        
        else if (cmd == 's') {
            if (mode == 1 ) {
                std::cout << "BLDC speed (1000=rev,1500=stop,2000=fwd): ";
                int pw;
                std::cin >> pw;
                bldc.setSpeed(pw);
            }
        }

        else if (cmd == 'x') {
            //stepper.stop();
            bldc.stop();
            std::cout << "Motors stopped.\n";
        }

        else if (cmd == 'q') {
            //stepper.stop();
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
