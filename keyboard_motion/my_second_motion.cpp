#include <iostream>
#include <array>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Keyboard input handler for non-blocking input
class Keyboard {
public:
    Keyboard() {
        tcgetattr(STDIN_FILENO, &original_termios_);
        termios new_termios = original_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    ~Keyboard() {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }

    int getKey() {
        char c = 0;
        int n = read(STDIN_FILENO, &c, 1);
        if (n > 0) {
            return c;
        }
        return -1;
    }

private:
    termios original_termios_;
};

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-ip>\n";
        return -1;
    }
    std::string robot_ip = argv[1];

    try {
        franka::Robot robot(robot_ip);

        // Shared atomic joint goal array
        std::array<std::atomic<double>, 7> q_goal_atomic;
        // Initialize q_goal_atomic to current robot joint positions
        auto initial_state = robot.readOnce();
        for (size_t i = 0; i < 7; ++i) {
            q_goal_atomic[i].store(initial_state.q[i]);
        }

        Keyboard keyboard;
        std::atomic<bool> running(true);

        // Helper lambda for atomic update with compare_exchange_weak
        auto update_atomic = [&](size_t idx, double delta) {
            double expected = q_goal_atomic[idx].load();
            double desired;
            do {
                desired = expected + delta;
            } while (!q_goal_atomic[idx].compare_exchange_weak(expected, desired));
        };

        // Keyboard input thread
        std::thread input_thread([&]() {
            const double increment = 0.03; // radians per key press
            std::cout << "Use keys to move joints:\n";
            std::cout << "Joint 1: q/Q (decrease/increase)\n";
            std::cout << "Joint 2: w/W\n";
            std::cout << "Joint 3: e/E\n";
            std::cout << "Joint 4: r/R\n";
            std::cout << "Joint 5: t/T\n";
            std::cout << "Joint 6: y/Y\n";
            std::cout << "Joint 7: u/U\n";
            std::cout << "Press 'x' to exit.\n";

            while (running.load()) {
                int key = keyboard.getKey();
                if (key != -1) {
                    switch (key) {
                        case 'q': update_atomic(0, -increment); break;
                        case 'Q': update_atomic(0, +increment); break;
                        case 'w': update_atomic(1, -increment); break;
                        case 'W': update_atomic(1, +increment); break;
                        case 'e': update_atomic(2, -increment); break;
                        case 'E': update_atomic(2, +increment); break;
                        case 'r': update_atomic(3, -increment); break;
                        case 'R': update_atomic(3, +increment); break;
                        case 't': update_atomic(4, -increment); break;
                        case 'T': update_atomic(4, +increment); break;
                        case 'y': update_atomic(5, -increment); break;
                        case 'Y': update_atomic(5, +increment); break;
                        case 'u': update_atomic(6, -increment); break;
                        case 'U': update_atomic(6, +increment); break;
                        case 'x':
                        case 'X':
                            std::cout << "Exit key pressed. Stopping control." << std::endl;
                            running.store(false);
                            return;
                        default:
                            break;
                    }
                    // Clamp joint goals between -pi and pi
                    for (size_t i = 0; i < 7; ++i) {
                        double val = q_goal_atomic[i].load();
                        if (val > M_PI) q_goal_atomic[i].store(M_PI);
                        else if (val < -M_PI) q_goal_atomic[i].store(-M_PI);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });

        try {
            robot.control([&](const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
                if (!running.load()) {
                    std::cout << "Exit requested from control loop." << std::endl;
                    throw std::runtime_error("Exit requested");
                }

                static std::array<double, 7> previous_tau = {0, 0, 0, 0, 0, 0, 0};
                static std::array<double, 7> smooth_q_goal = {0, 0, 0, 0, 0, 0, 0};
                const double max_goal_increment = 0.01;  // radians per control step

                // Smoothly interpolate joint goals
                for (size_t i = 0; i < 7; ++i) {
                    double desired = q_goal_atomic[i].load();
                    double diff = desired - smooth_q_goal[i];
                    if (diff > max_goal_increment) diff = max_goal_increment;
                    else if (diff < -max_goal_increment) diff = -max_goal_increment;
                    smooth_q_goal[i] += diff;
                }

                std::array<double, 7> tau{};
                const double kp = 2.0;
                const double kd = 0.5;
                const double alpha = 0.1;  // torque smoothing factor
                const double max_tau = 5.0; // max torque limit

                for (size_t i = 0; i < 7; ++i) {
                    double error = smooth_q_goal[i] - state.q[i];
                    double d_error = -state.dq[i];
                    double tau_desired = kp * error + kd * d_error;

                    // Smooth torque command to avoid sudden jumps
                    tau[i] = alpha * tau_desired + (1.0 - alpha) * previous_tau[i];

                    // Clamp torque commands to safe limits
                    if (tau[i] > max_tau) tau[i] = max_tau;
                    if (tau[i] < -max_tau) tau[i] = -max_tau;

                    previous_tau[i] = tau[i];
                }
                return franka::Torques(tau);
            });
        } catch (const std::exception& ex) {
            std::cout << "Control stopped: " << ex.what() << std::endl;
        }

        input_thread.join();

        std::cout << "Control finished, program exiting." << std::endl;

    } catch (const franka::Exception& ex) {
        std::cerr << "Franka Exception: " << ex.what() << std::endl;
        return -1;
    } catch (const std::exception& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return -1;
    }

    return 0;
}

