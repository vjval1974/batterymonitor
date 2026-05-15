// Browser port of the firmware's pure-math layer.
//
// Mirrors include/Config.h, include/CurrentMath.h, include/SocLookup.h,
// include/StateOfCharge.h, include/EnergyAccumulator.h, and the screen
// layouts in src/Display.cpp. Keep this file in lockstep with those headers
// when changing calibration constants or display formats.
//
// The simulator drives this module the same way main.cpp drives the firmware:
//   - takeSample() every kMeasurementPeriodMs ms
//   - renderScreen() every kDisplayPeriodMs ms
// Buttons are injected through pressButton().

export const Config = {
    cal: {
        acs712VoltsPerAmp: 0.066,
        adcMidpointMv: 1650.0,
        adcMaxCount: 1023,
        batteryDividerVmax: 15.88,
        solarOffsetA: 0.05,
        acOffsetA: 0.00,
        carOffsetA: 0.24,
        drawOffsetA: 0.03,
        currentNoiseFloorA: 0.18,
        relaySettleMs: 200,
        relayReleaseMs: 100,
    },
    battery: {
        capacityAh: 100.0,
        voltageFull: 13.5,
        voltageNominal: 12.6,
        voltageLowAlarm: 11.8,
        voltageCritical: 11.0,
    },
    timing: {
        measurementPeriodMs: 5000,
        displayPeriodMs: 53,
        buttonDebounceMs: 40,
        get measurementWindowSec() {
            return (this.measurementPeriodMs + Config.cal.relaySettleMs +
                    Config.cal.relayReleaseMs) / 1000;
        },
    },
    logger: {
        maxLogBytes: 512 * 1024,
    },
    ui: {
        backlightOff: 'off',
        backlightCharging: 'green',
        backlightIdle: 'blue',
        backlightDischarging: 'white',
        backlightWarning: 'yellow',
        backlightCritical: 'red',
    },
};

// ---------------------------------------------------------------------------
// Pure math
// ---------------------------------------------------------------------------

const mapLinear = (x, inLo, inHi, outLo, outHi) =>
    (x - inLo) * (outHi - outLo) / (inHi - inLo) + outLo;

export function adcToCurrent(adcValue, offset) {
    const { acs712VoltsPerAmp, adcMidpointMv, adcMaxCount, currentNoiseFloorA } = Config.cal;
    const mv = mapLinear(adcValue, 0, adcMaxCount, -adcMidpointMv, adcMidpointMv);
    const amps = (mv / acs712VoltsPerAmp) / 1000.0;
    const corrected = amps + offset;
    if (Math.abs(corrected) <= currentNoiseFloorA) return 0.0;
    return corrected;
}

export function adcToBatteryVolts(adcValue) {
    return mapLinear(adcValue, 0, Config.cal.adcMaxCount, 0,
                     Config.cal.batteryDividerVmax);
}

const SOC_CURVE = [
    [10.50,   0],
    [11.31,  10],
    [11.58,  20],
    [11.75,  30],
    [11.90,  40],
    [12.06,  50],
    [12.20,  60],
    [12.32,  70],
    [12.42,  80],
    [12.50,  90],
    [12.70, 100],
];

export function voltageToSoc(volts) {
    if (volts <= SOC_CURVE[0][0]) return 0;
    if (volts >= SOC_CURVE[SOC_CURVE.length - 1][0]) return 100;
    for (let i = 1; i < SOC_CURVE.length; i++) {
        if (volts <= SOC_CURVE[i][0]) {
            const [vLo, sLo] = SOC_CURVE[i - 1];
            const [vHi, sHi] = SOC_CURVE[i];
            const t = (volts - vLo) / (vHi - vLo);
            return sLo + t * (sHi - sLo);
        }
    }
    return 100;
}

// ---------------------------------------------------------------------------
// Stateful components (mirrors of StateOfCharge.h / EnergyAccumulator.h /
// RingBuffer.h)
// ---------------------------------------------------------------------------

export class StateOfCharge {
    constructor(capacityAh = Config.battery.capacityAh) {
        this.capacityAh = capacityAh;
        this.socPct = 50;
        this.reanchorAccumSec = 0;
    }
    seedFromVoltage(v) { this.socPct = voltageToSoc(v); }
    update(netCurrentA, dtSec) {
        const deltaAh = (netCurrentA * dtSec) / 3600;
        const deltaPct = (deltaAh / this.capacityAh) * 100;
        this.socPct -= deltaPct;
        this.socPct = Math.max(0, Math.min(100, this.socPct));
    }
    maybeReanchor(volts, netCurrentA, dtSec) {
        if (volts >= Config.battery.voltageFull && Math.abs(netCurrentA) < 0.5) {
            this.reanchorAccumSec += dtSec;
            if (this.reanchorAccumSec >= 300) {
                this.socPct = 100;
                this.reanchorAccumSec = 0;
            }
        } else {
            this.reanchorAccumSec = 0;
        }
    }
}

export class EnergyAccumulator {
    constructor() { this.wh = 0; }
    accumulate(watts, dtSec) { this.wh += (watts * dtSec) / 3600; }
}

export class RingBuffer {
    constructor(n) { this.n = n; this.data = []; }
    push(v) {
        this.data.push(v);
        if (this.data.length > this.n) this.data.shift();
    }
    min() { return this.data.length ? Math.min(...this.data) : 0; }
    max() { return this.data.length ? Math.max(...this.data) : 0; }
    mean() {
        if (!this.data.length) return 0;
        return this.data.reduce((a, b) => a + b, 0) / this.data.length;
    }
}

// ---------------------------------------------------------------------------
// Sampler — reads the simulated ADC values supplied by the simulator and
// produces a Sample identical to what the firmware would produce.
// ---------------------------------------------------------------------------

export function takeSample(adc, soc, drawEnergy, solarEnergy) {
    const { cal, timing } = Config;
    const volts = adcToBatteryVolts(adc.battery);
    const solarA = -adcToCurrent(adc.solar, cal.solarOffsetA);
    const acA    = -adcToCurrent(adc.ac,    cal.acOffsetA);
    const carA   = -adcToCurrent(adc.car,   cal.carOffsetA);
    const drawA  =  adcToCurrent(adc.draw,  cal.drawOffsetA);
    const chargeA = solarA + acA + carA;
    const netA = drawA - chargeA;

    const drawW = volts * drawA;
    const solarW = volts * solarA;

    soc.update(netA, timing.measurementWindowSec);
    soc.maybeReanchor(volts, netA, timing.measurementWindowSec);
    drawEnergy.accumulate(drawW, timing.measurementWindowSec);
    solarEnergy.accumulate(solarW, timing.measurementWindowSec);

    return {
        batteryVolts: volts,
        drawA, solarA, acA, carA,
        chargeA, netA,
        drawW, solarW,
        energyWh: drawEnergy.wh,
        solarEnergyWh: solarEnergy.wh,
        socPct: soc.socPct,
    };
}

// ---------------------------------------------------------------------------
// Backlight state machine (mirrors Display::updateBacklight)
// ---------------------------------------------------------------------------

export function backlightFor(sample) {
    const { battery, ui } = Config;
    if (sample.batteryVolts <= battery.voltageCritical) return ui.backlightCritical;
    if (sample.batteryVolts <= battery.voltageLowAlarm) return ui.backlightWarning;
    if (sample.chargeA > 0.5) return ui.backlightCharging;
    if (sample.drawA  > 0.5) return ui.backlightDischarging;
    return ui.backlightIdle;
}

// ---------------------------------------------------------------------------
// Screen renderer — mirrors Display::render*() in src/Display.cpp.
// Each function returns { top, bottom } strings padded to 16 chars.
// ---------------------------------------------------------------------------

const pad16 = (s) => s.length >= 16 ? s.slice(0, 16) : s + ' '.repeat(16 - s.length);

function fmt(num, width, prec) {
    const s = num.toFixed(prec);
    return s.length >= width ? s : ' '.repeat(width - s.length) + s;
}

export const SCREENS = [
    {
        name: 'Summary',
        render: (s) => {
            const top = `${fmt(s.batteryVolts, 5, 2)}V  SOC${fmt(s.socPct, 3, 0)}%`;
            const dir = s.netA >= 0 ? 'OUT' : 'IN ';
            const mag = Math.abs(s.netA);
            const bot = `${dir}${fmt(mag, 5, 2)}A ${fmt(s.drawW, 4, 0)}W`;
            return { top: pad16(top), bottom: pad16(bot) };
        }
    },
    {
        name: 'Currents',
        render: (s) => ({
            top: pad16(`Sol${fmt(s.solarA, 5, 2)} Car${fmt(s.carA, 5, 2)}`),
            bottom: pad16(`AC ${fmt(s.acA, 5, 2)} Ld ${fmt(s.drawA, 5, 2)}`),
        })
    },
    {
        name: 'Power',
        render: (s) => ({
            top: pad16(`Load    ${fmt(s.drawW, 6, 1)}W`),
            bottom: pad16(`Solar   ${fmt(s.solarW, 6, 1)}W`),
        })
    },
    {
        name: 'Energy',
        render: (s) => ({
            top: pad16(`Drawn  ${fmt(s.energyWh, 7, 1)}Wh`),
            bottom: pad16(`SOC      ${fmt(s.socPct, 5, 1)}%`),
        })
    },
    {
        name: 'Min/Max V',
        render: (s, ctx) => ({
            top: pad16(`Vmin ${fmt(ctx.history.min(), 5, 2)}V`),
            bottom: pad16(`Vmax ${fmt(ctx.history.max(), 5, 2)}V`),
        })
    },
    {
        name: 'Logger',
        render: (s, ctx) => ({
            top: pad16(`Log ${ctx.logName}`),
            bottom: pad16(`${ctx.logBytes.toString().padStart(6)} bytes`),
        })
    },
    {
        name: 'About',
        render: (s, ctx) => {
            const up = ctx.uptimeSec;
            const d = Math.floor(up / 86400);
            const h = Math.floor(up / 3600) % 24;
            const m = Math.floor(up / 60) % 60;
            return {
                top: pad16(`fw ${ctx.fwVersion}`),
                bottom: pad16(`up ${d}d${String(h).padStart(2,'0')}h${String(m).padStart(2,'0')}m`),
            };
        }
    },
];

// CSV log row — matches Logger::record() format in src/Logger.cpp.
export function csvRow(timestampIso, s) {
    return [
        timestampIso,
        s.batteryVolts.toFixed(2),
        s.drawA.toFixed(3),
        s.solarA.toFixed(3),
        s.acA.toFixed(3),
        s.carA.toFixed(3),
        s.chargeA.toFixed(3),
        s.netA.toFixed(3),
        s.drawW.toFixed(2),
        s.solarW.toFixed(2),
        s.energyWh.toFixed(2),
        s.socPct.toFixed(1),
    ].join(',');
}

export const CSV_HEADER =
    'iso_time,vbatt,iload,isolar,iac,icar,ichg_total,inet,pload,psolar,energy_wh,soc_pct';
