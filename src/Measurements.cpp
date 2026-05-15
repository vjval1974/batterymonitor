#include "Measurements.h"

#ifdef TARGET_AVR
#    include <avr/pgmspace.h>
#else
#    define PROGMEM
#    define strcpy_P(dest, src) strcpy((dest), (src))
#endif

#include <stddef.h>

namespace bm {
namespace {

struct FieldMeta
{
    const char* label;
    const char* unit;
};

// Labels are constrained to 14 characters so they fit on the LCD next to the
// short "page x/y" indicator.
const FieldMeta kMeta[kFieldCount] = {
    {"Battery V",     "V"},
    {"Load I",        "A"},
    {"Solar I",       "A"},
    {"AC chg I",      "A"},
    {"Car chg I",     "A"},
    {"Charge total",  "A"},
    {"Net current",   "A"},
    {"Load power",    "W"},
    {"Solar power",   "W"},
    {"Energy drawn",  "Wh"},
    {"State of chg",  "%"},
};

} // namespace

const char* labelFor(Field f)
{
    const uint8_t i = static_cast<uint8_t>(f);
    if (i >= kFieldCount)
    {
        return "?";
    }
    return kMeta[i].label;
}

const char* unitFor(Field f)
{
    const uint8_t i = static_cast<uint8_t>(f);
    if (i >= kFieldCount)
    {
        return "";
    }
    return kMeta[i].unit;
}

} // namespace bm
