#include "localization/ekf_telemetry.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "drivetrain.hpp"
#include "localization/ekf_config.hpp"
#include "localization/localizer.hpp"
#include "localization/noise_meter.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace loc {

namespace {
using namespace ekf_config;

// Cycles controller pages: main -> IMU -> FRONT -> BACK -> LEFT -> RIGHT noise pages -> main.
// Driver control uses RIGHT, DOWN and Y; X is free.
constexpr auto PAGE_BUTTON = pros::E_CONTROLLER_DIGITAL_X;

// Position std dev in inches: sqrt of the larger axis variance (worst direction, roughly).
double positionStd(const LocalizerStatus& s) { return std::sqrt(std::max(s.P.a, s.P.d)); }

void printCsvHeader() {
    std::printf("ekf,ms,ekf_x,ekf_y,odom_x,odom_y,std");
    for (const auto& sensor : SENSORS)
        std::printf(",%s_meas,%s_exp,%s_r,%s_status", sensor.name, sensor.name, sensor.name, sensor.name);
    std::printf("\n");
}

// One CSV line. Status is OK when accepted, otherwise the reject code (RES, ANG, ...).
// -1 in meas/exp means "no value" (nothing seen / ray hit nothing).
void printCsvLine(const LocalizerStatus& s) {
    std::printf("ekf,%lu,%.2f,%.2f,%.2f,%.2f,%.3f", static_cast<unsigned long>(pros::millis()), s.ekf.x,
                s.ekf.y, s.odom.x, s.odom.y, positionStd(s));
    for (const auto& r : s.sensors)
        std::printf(",%.2f,%.2f,%.3f,%s", r.measuredIn, r.expectedIn, r.r, rejectCode(r.reason));
    std::printf("\n");
}

// Controller lines are padded to a fixed width so a shorter value overwrites the old text.
// Main page:
//   Line 1: EKF X/Y while the EKF is correcting; otherwise "oX" = LemLib odometry X/Y
//           (EKF disabled, not started, or setEnabled(false)). IMU failure warning overrides it.
//   Line 2: which sensors' last reading was used, or EKF state
//   Line 3: heading and position std dev
void printMainPage(pros::Controller& controller, int line, const LocalizerStatus& s, bool imuOk) {
    const lemlib::Pose pose = chassis.getPose();
    const bool correcting = s.running && s.enabled;
    switch (line) {
        case 0:
            if (!imuOk) controller.print(0, 0, "IMU CAL FAILED!   ");
            else if (correcting) controller.print(0, 0, "X:%6.1f Y:%6.1f  ", s.ekf.x, s.ekf.y);
            else controller.print(0, 0, "oX:%6.1f Y:%6.1f ", pose.x, pose.y);
            break;
        case 1:
            if (!EKF_ENABLED) controller.print(1, 0, "EKF disabled      ");
            else if (!s.running) controller.print(1, 0, "EKF not started   ");
            else {
                auto yn = [&](int i) { return s.sensors[i].accepted() ? 'Y' : 'N'; };
                controller.print(1, 0, "%c:%c %c:%c %c:%c %c:%c%s", SENSORS[0].name[0], yn(0), SENSORS[1].name[0],
                                 yn(1), SENSORS[2].name[0], yn(2), SENSORS[3].name[0], yn(3),
                                 s.enabled ? "  " : " off");
            }
            break;
        default:
            if (s.running) controller.print(2, 0, "H:%7.1f SD:%4.2f ", pose.theta, positionStd(s));
            else controller.print(2, 0, "H:%7.1f          ", pose.theta);
            break;
    }
}

// IMU page (IMU_SCALE tuning, EKF_TUNING.md step 1). Total rotation, unwrapped, so 10 turns
// reads ~3600 with no wrap counting.
//   Line 1: title       Line 2: raw IMU rotation       Line 3: scaled (raw * IMU_SCALE)
void printImuPage(pros::Controller& controller, int line) {
    switch (line) {
        case 0: controller.print(0, 0, "IMU total rotation"); break;
        case 1: controller.print(1, 0, "raw: %9.1f    ", imuRawRotation()); break;
        default: controller.print(2, 0, "scl: %9.1f    ", imuScaledRotation()); break;
    }
}

// Noise page for one sensor (R tuning, EKF_TUNING.md step 5):
//   Line 1: sensor name and how full the window is
//   Line 2: std dev and bias (inches)
//   Line 3: average distance and the R to put in ekf_config (only once the window is full)
void printNoisePage(pros::Controller& controller, int line, const LocalizerStatus& s, int sensor) {
    const NoiseSummary& n = s.noise[sensor];
    switch (line) {
        case 0:
            controller.print(0, 0, "%-5s n:%3u/%3u   ", SENSORS[sensor].name, static_cast<unsigned>(n.count),
                             static_cast<unsigned>(NoiseMeter::WINDOW));
            break;
        case 1:
            if (!s.running) controller.print(1, 0, "EKF not started   ");
            else if (n.count < 2) controller.print(1, 0, "no readings       ");
            else controller.print(1, 0, "sd%5.3f b%4.2f    ", n.stdDev, n.bias);
            break;
        default:
            if (n.full) controller.print(2, 0, "d%5.1f R:%6.3f  ", n.mean, n.r);
            else controller.print(2, 0, "wait...           ");
            break;
    }
}
} // namespace

void startEkfTelemetry(bool imuOk) {
    if (!TERMINAL_TELEMETRY && !CONTROLLER_TELEMETRY) return;
    static pros::Task task(
        [imuOk] {
            pros::Controller controller(pros::E_CONTROLLER_MASTER);
            if (CONTROLLER_TELEMETRY) {
                controller.clear();
                pros::delay(CONTROLLER_LINE_MS); // space controller commands apart
            }
            if (TERMINAL_TELEMETRY) printCsvHeader();

            int line = 0;
            // 0 = main, 1 = IMU, 2-5 = noise page for SENSORS[page - FIRST_SENSOR_PAGE]
            constexpr int FIRST_SENSOR_PAGE = 2, PAGES = FIRST_SENSOR_PAGE + 4;
            int page = 0;
            std::uint32_t lastCsv = 0;
            std::uint32_t wake = pros::millis();
            while (true) {
                const LocalizerStatus s = localizer.getStatus();
                if (CONTROLLER_TELEMETRY) {
                    if (controller.get_digital_new_press(PAGE_BUTTON)) {
                        page = (page + 1) % PAGES;
                        line = 0; // redraw the new page from the top
                    }
                    // One controller command per period.
                    if (page == 0) printMainPage(controller, line, s, imuOk);
                    else if (page == 1) printImuPage(controller, line);
                    else printNoisePage(controller, line, s, page - FIRST_SENSOR_PAGE);
                    line = (line + 1) % 3;
                }
                if (TERMINAL_TELEMETRY && s.running && pros::millis() - lastCsv >= CSV_PERIOD_MS) {
                    lastCsv = pros::millis();
                    printCsvLine(s);
                }
                pros::Task::delay_until(&wake, CONTROLLER_LINE_MS);
            }
        },
        TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "EKF telemetry");
}

} // namespace loc
