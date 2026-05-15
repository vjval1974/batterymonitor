#include "Display.h"

#include "Config.h"
#include "Logger.h"

#include <stdio.h>
#include <string.h>

#ifdef TARGET_AVR
#    include <Adafruit_RGBLCDShield.h>
#    include <Arduino.h>
static Adafruit_RGBLCDShield s_lcd = Adafruit_RGBLCDShield();
#endif

namespace bm {

namespace {
constexpr uint8_t kPageCount = 7;

void lcdClear()
{
#ifdef TARGET_AVR
    s_lcd.clear();
#endif
}

void lcdSet(uint8_t col, uint8_t row, const char* s)
{
#ifdef TARGET_AVR
    s_lcd.setCursor(col, row);
    s_lcd.print(s);
#else
    (void) col;
    (void) row;
    (void) s;
#endif
}

void lcdBacklight(uint8_t colour)
{
#ifdef TARGET_AVR
    s_lcd.setBacklight(colour);
#else
    (void) colour;
#endif
}

uint32_t now_ms()
{
#ifdef TARGET_AVR
    return millis();
#else
    return 0;
#endif
}

} // namespace

Display::Display()
    : page_(0), lastPage_(255), pendingButton_(Button::None), lastButtonMs_(0),
      voltageHistoryDirty_(true)
{
}

void Display::begin()
{
#ifdef TARGET_AVR
    s_lcd.begin(16, 2);
    s_lcd.setBacklight(config::ui::kBacklightIdle);
    s_lcd.print(F("Battery Monitor"));
    s_lcd.setCursor(0, 1);
    s_lcd.print(F("fw " FW_VERSION));
    delay(1500);
    s_lcd.clear();
#endif
}

Button Display::readButtons()
{
#ifdef TARGET_AVR
    const uint8_t raw = s_lcd.readButtons();
    if (raw & BUTTON_UP) return Button::Up;
    if (raw & BUTTON_DOWN) return Button::Down;
    if (raw & BUTTON_LEFT) return Button::Left;
    if (raw & BUTTON_RIGHT) return Button::Right;
    if (raw & BUTTON_SELECT) return Button::Select;
#endif
    if (pendingButton_ != Button::None)
    {
        const Button b = pendingButton_;
        pendingButton_ = Button::None;
        return b;
    }
    return Button::None;
}

void Display::injectButton(Button b) { pendingButton_ = b; }

void Display::tick(const Sample& s, const Logger& logger, uint32_t uptimeSec)
{
    const uint32_t t = now_ms();
    const Button b = readButtons();
    if (b != Button::None && (t - lastButtonMs_) >= config::timing::kButtonDebounceMs)
    {
        lastButtonMs_ = t;
        switch (b)
        {
        case Button::Left:
            page_ = (page_ == 0) ? (kPageCount - 1) : (page_ - 1);
            break;
        case Button::Right:
            page_ = (page_ + 1) % kPageCount;
            break;
        case Button::Select:
            // Reserved: future "rotate log" action wired in main.
            break;
        default:
            break;
        }
    }

    if (voltageHistoryDirty_)
    {
        voltageHistory_.push(s[Field::BatteryVolts]);
        voltageHistoryDirty_ = false;
    }

    updateBacklight(s);

    if (page_ != lastPage_)
    {
        lcdClear();
        lastPage_ = page_;
    }

    switch (page_)
    {
    case 0: renderSummary(s); break;
    case 1: renderCurrents(s); break;
    case 2: renderPower(s); break;
    case 3: renderEnergy(s); break;
    case 4: renderMinMaxV(); break;
    case 5: renderLogger(logger); break;
    case 6: renderAbout(uptimeSec); break;
    default: page_ = 0; break;
    }
}

void Display::updateBacklight(const Sample& s)
{
    using namespace config;
    uint8_t c = ui::kBacklightIdle;
    if (s[Field::BatteryVolts] <= battery::kVoltageCritical)
    {
        c = ui::kBacklightCritical;
    }
    else if (s[Field::BatteryVolts] <= battery::kVoltageLowAlarm)
    {
        c = ui::kBacklightWarning;
    }
    else if (s[Field::TotalChargeCurrentA] > 0.5f)
    {
        c = ui::kBacklightCharging;
    }
    else if (s[Field::DrawCurrentA] > 0.5f)
    {
        c = ui::kBacklightDischarging;
    }
    lcdBacklight(c);
}

void Display::renderSummary(const Sample& s)
{
    char top[17];
    char bot[17];
    snprintf(top, sizeof(top), "%5.2fV  SOC%3.0f%%", static_cast<double>(s[Field::BatteryVolts]),
             static_cast<double>(s[Field::SocPct]));
    const float net = s[Field::NetCurrentA];
    snprintf(bot, sizeof(bot), "%s%5.2fA %4.0fW", net >= 0 ? "OUT" : "IN ",
             static_cast<double>(net < 0 ? -net : net),
             static_cast<double>(s[Field::DrawPowerW]));
    lcdSet(0, 0, top);
    lcdSet(0, 1, bot);
}

void Display::renderCurrents(const Sample& s)
{
    char top[17];
    char bot[17];
    snprintf(top, sizeof(top), "Sol%5.2f Car%5.2f", static_cast<double>(s[Field::SolarCurrentA]),
             static_cast<double>(s[Field::CarCurrentA]));
    snprintf(bot, sizeof(bot), "AC %5.2f Ld %5.2f", static_cast<double>(s[Field::AcCurrentA]),
             static_cast<double>(s[Field::DrawCurrentA]));
    lcdSet(0, 0, top);
    lcdSet(0, 1, bot);
}

void Display::renderPower(const Sample& s)
{
    char top[17];
    char bot[17];
    snprintf(top, sizeof(top), "Load    %6.1fW", static_cast<double>(s[Field::DrawPowerW]));
    snprintf(bot, sizeof(bot), "Solar   %6.1fW", static_cast<double>(s[Field::SolarPowerW]));
    lcdSet(0, 0, top);
    lcdSet(0, 1, bot);
}

void Display::renderEnergy(const Sample& s)
{
    char top[17];
    char bot[17];
    snprintf(top, sizeof(top), "Drawn  %7.1fWh", static_cast<double>(s[Field::EnergyDrawnWh]));
    snprintf(bot, sizeof(bot), "SOC      %5.1f%%", static_cast<double>(s[Field::SocPct]));
    lcdSet(0, 0, top);
    lcdSet(0, 1, bot);
}

void Display::renderMinMaxV()
{
    char top[17];
    char bot[17];
    snprintf(top, sizeof(top), "Vmin %5.2fV", static_cast<double>(voltageHistory_.min()));
    snprintf(bot, sizeof(bot), "Vmax %5.2fV", static_cast<double>(voltageHistory_.max()));
    lcdSet(0, 0, top);
    lcdSet(0, 1, bot);
}

void Display::renderLogger(const Logger& logger)
{
    char top[17];
    char bot[17];
    if (logger.ready())
    {
        snprintf(top, sizeof(top), "Log %s", logger.currentFile());
        snprintf(bot, sizeof(bot), "%6lu bytes",
                 static_cast<unsigned long>(logger.bytesWritten()));
    }
    else
    {
        snprintf(top, sizeof(top), "Log: SD ERROR");
        snprintf(bot, sizeof(bot), "card missing?");
    }
    lcdSet(0, 0, top);
    lcdSet(0, 1, bot);
}

void Display::renderAbout(uint32_t uptimeSec)
{
    char top[17];
    char bot[17];
    snprintf(top, sizeof(top), "fw %s", FW_VERSION);
    const uint32_t d = uptimeSec / 86400;
    const uint32_t h = (uptimeSec / 3600) % 24;
    const uint32_t m = (uptimeSec / 60) % 60;
    snprintf(bot, sizeof(bot), "up %lud%02luh%02lum", static_cast<unsigned long>(d),
             static_cast<unsigned long>(h), static_cast<unsigned long>(m));
    lcdSet(0, 0, top);
    lcdSet(0, 1, bot);
}

} // namespace bm
