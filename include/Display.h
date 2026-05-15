#pragma once
//
// 16x2 LCD UI with debounced 5-button navigation and multiple screens.
//
// Screens
//   0  Summary       VV.VV V  / +SS.S% SOC
//   1  Currents      ↑c.cc ↓d.dd
//   2  Power         load WW.W solar SS.S
//   3  Energy        drawn  XXXX Wh
//   4  Min/Max V     L 11.85 H 13.92
//   5  Logger        file YYMMDDHH .CSV  (or warning if degraded)
//   6  About         FW + uptime
//
// Backlight colour reflects battery state (green=charging, yellow=warning,
// red=critical, white=discharging, blue=idle).

#include "Measurements.h"
#include "RingBuffer.h"

#include <stdint.h>

namespace bm {

class Logger;

enum class Button : uint8_t
{
    None = 0,
    Up,
    Down,
    Left,
    Right,
    Select
};

class Display
{
public:
    Display();

    void begin();

    // Called from the display thread on its own cadence. Reads buttons, updates
    // the active screen, and changes backlight colour based on the sample.
    void tick(const Sample& s, const Logger& logger, uint32_t uptimeSec);

    // Inject a button event from outside (used by the simulator).
    void injectButton(Button b);

private:
    void renderSummary(const Sample& s);
    void renderCurrents(const Sample& s);
    void renderPower(const Sample& s);
    void renderEnergy(const Sample& s);
    void renderMinMaxV();
    void renderLogger(const Logger& logger);
    void renderAbout(uint32_t uptimeSec);

    void updateBacklight(const Sample& s);
    Button readButtons();

    uint8_t           page_;
    uint8_t           lastPage_;
    Button            pendingButton_;
    uint32_t          lastButtonMs_;
    RingBuffer<60>    voltageHistory_; // ~5 min at 5-s sampling
    bool              voltageHistoryDirty_;
};

} // namespace bm
