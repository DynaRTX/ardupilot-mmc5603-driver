/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_MMC5XX3_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_MMC5xx3_I2C_ADDR
# define HAL_COMPASS_MMC5xx3_I2C_ADDR 0x30
#endif

class AP_Compass_MMC5XX3 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "MMC5xx3";

private:
    AP_Compass_MMC5XX3(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    // Which chip variant was detected at init()
    enum class ChipVariant : uint8_t {
        MMC5983,  // 16-bit output, 6 data bytes, regs at 0x09/0x0A/0x0B/0x08
        MMC5603,  // 20-bit output, 9 data bytes, regs at 0x1B/0x1C/0x1D/0x18
    } chip_variant;

    /*
     * State machine used by timer().
     *
     * Every ~10 ms (100 Hz timer) the state machine advances one step.
     * The full SET/RESET calibration cycle runs every DEGAUSS_INTERVAL
     * measurements (~3 s at 100 Hz).
     *
     * Flow:
     *   STATE_SET
     *     └─ write CTRL0_SET, arm state_start_ms
     *   STATE_SET_MEASURE          (waits ≥ SET_RESET_DELAY_MS for pulse settling)
     *     └─ write TM_M, arm state_start_ms
     *   STATE_SET_WAIT             (waits for Meas_M_Done, with timeout)
     *     └─ read data0, write CTRL0_RESET, arm state_start_ms
     *   STATE_RESET_MEASURE        (waits ≥ SET_RESET_DELAY_MS for pulse settling)
     *     └─ write TM_M, arm state_start_ms
     *   STATE_RESET_WAIT           (waits for Meas_M_Done, with timeout)
     *     └─ compute field/offset, write TM_M, arm state_start_ms
     *   STATE_MEASURE              (repeated high-rate measurements, with timeout)
     *     └─ after DEGAUSS_INTERVAL → back to STATE_SET
     */
    enum class MMCState : uint8_t {
        STATE_SET,           // Issue SET coil pulse
        STATE_SET_MEASURE,   // Wait for SET settling, then start measurement
        STATE_SET_WAIT,      // Wait for magnetic measurement to complete (after SET)
        STATE_RESET_MEASURE, // Wait for RESET settling, then start measurement
        STATE_RESET_WAIT,    // Wait for magnetic measurement to complete (after RESET)
        STATE_MEASURE,       // Continuous high-rate measurements between degauss cycles
    } state;

    bool init();
    void timer();

    bool force_external;
    bool have_initial_offset;
    enum Rotation rotation;

    // Running bridge-offset estimate (SET+RESET average, low-pass filtered)
    Vector3f offset;

    // Measurement counter; wraps every DEGAUSS_INTERVAL to trigger SET/RESET cycle
    uint16_t measure_count;

    // Timestamp (AP_HAL::millis()) when the current state was entered.
    // Used for:
    //   • SET/RESET pulse settling guard (≥ SET_RESET_DELAY_MS before TM_M)
    //   • Measurement-done timeout (bail out if chip stalls)
    uint32_t state_start_ms;

    // 9-byte buffer: accommodates MMC5603 20-bit (9-byte) reads.
    // MMC5983 only uses the first 6 bytes.
    uint8_t data0[9];
};

#endif  // AP_COMPASS_MMC5XX3_ENABLED
