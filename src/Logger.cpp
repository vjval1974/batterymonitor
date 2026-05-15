#include "Logger.h"

#include "Config.h"

#include <stdio.h>
#include <string.h>

#ifdef TARGET_AVR
#    include <Arduino.h>
#endif

namespace bm {

namespace {
constexpr char kCsvHeader[] =
    "iso_time,vbatt,iload,isolar,iac,icar,ichg_total,inet,pload,psolar,energy_wh,soc_pct";
}

Logger::Logger() : ready_(false), bytesWritten_(0)
{
    currentName_[0] = '\0';
}

void Logger::formatName(const DateTimeFields& now, char* out, size_t n)
{
    if (now.year == 0)
    {
        snprintf(out, n, "NORTC.CSV");
        return;
    }
    // 8.3-compliant: YYMMDDHH.CSV
    snprintf(out, n, "%02u%02u%02u%02u.CSV", static_cast<unsigned>(now.year % 100),
             static_cast<unsigned>(now.month), static_cast<unsigned>(now.day),
             static_cast<unsigned>(now.hour));
}

bool Logger::begin(const DateTimeFields& now)
{
#ifdef TARGET_AVR
    using namespace config::pin;
    if (!SD.begin(kSdChipSelect, kSdMosi, kSdMiso, kSdSck))
    {
        ready_ = false;
        return false;
    }
#endif
    return openFor(now);
}

bool Logger::openFor(const DateTimeFields& now)
{
    formatName(now, currentName_, sizeof(currentName_));

#ifdef TARGET_AVR
    if (file_)
    {
        file_.close();
    }
    file_ = SD.open(currentName_, FILE_WRITE);
    if (!file_)
    {
        ready_ = false;
        return false;
    }
    if (file_.size() == 0)
    {
        file_.println(kCsvHeader);
    }
    bytesWritten_ = file_.size();
#else
    bytesWritten_ = 0;
#endif
    ready_ = true;
    return true;
}

bool Logger::rotate(const DateTimeFields& now) { return openFor(now); }

bool Logger::record(const DateTimeFields& now, const Sample& s)
{
    if (!ready_)
    {
        return false;
    }

    char line[160];
    int n = snprintf(line, sizeof(line),
                     "%04u-%02u-%02uT%02u:%02u:%02u,"
                     "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f",
                     static_cast<unsigned>(now.year), static_cast<unsigned>(now.month),
                     static_cast<unsigned>(now.day), static_cast<unsigned>(now.hour),
                     static_cast<unsigned>(now.minute), static_cast<unsigned>(now.second),
                     static_cast<double>(s[Field::BatteryVolts]),
                     static_cast<double>(s[Field::DrawCurrentA]),
                     static_cast<double>(s[Field::SolarCurrentA]),
                     static_cast<double>(s[Field::AcCurrentA]),
                     static_cast<double>(s[Field::CarCurrentA]),
                     static_cast<double>(s[Field::TotalChargeCurrentA]),
                     static_cast<double>(s[Field::NetCurrentA]),
                     static_cast<double>(s[Field::DrawPowerW]),
                     static_cast<double>(s[Field::SolarPowerW]),
                     static_cast<double>(s[Field::EnergyDrawnWh]),
                     static_cast<double>(s[Field::SocPct]));
    if (n <= 0)
    {
        return false;
    }

#ifdef TARGET_AVR
    file_.println(line);
    file_.flush();
    bytesWritten_ = file_.size();
    if (bytesWritten_ >= config::logger::kMaxLogBytes)
    {
        return rotate(now);
    }
#else
    bytesWritten_ += static_cast<uint32_t>(n) + 1;
#endif
    return true;
}

} // namespace bm
