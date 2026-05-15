#pragma once
//
// SD-card logging in CSV format with size-based file rotation.
//
// On boot, the logger creates a fresh file named `YYMMDDHH.CSV` (8.3) and
// writes a header row. Subsequent `record()` calls append one row per sample.
// When the active file exceeds Config::logger::kMaxLogBytes a new file is
// opened with the current timestamp.
//
// Failure modes are designed to be non-fatal: if the SD card is missing or
// stops responding, the firmware keeps measuring and displaying — it just sets
// an internal "degraded" flag so the UI can surface the problem.

#include "Measurements.h"

#include <stdint.h>

#ifdef TARGET_AVR
#    include <SD.h>
#endif

namespace bm {

struct DateTimeFields
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
};

class Logger
{
public:
    Logger();

    // Returns true if the SD card initialised and the first log file opened.
    bool begin(const DateTimeFields& now);

    // Append a row. The timestamp is taken from `now`; pass `{}` if no RTC.
    bool record(const DateTimeFields& now, const Sample& s);

    // Force-rotate to a new file (e.g. from a menu action).
    bool rotate(const DateTimeFields& now);

    bool      ready() const { return ready_; }
    uint32_t  bytesWritten() const { return bytesWritten_; }
    const char* currentFile() const { return currentName_; }

private:
    bool openFor(const DateTimeFields& now);
    void formatName(const DateTimeFields& now, char* out, size_t n);

    bool     ready_;
    uint32_t bytesWritten_;
    char     currentName_[13];

#ifdef TARGET_AVR
    File file_;
#endif
};

} // namespace bm
