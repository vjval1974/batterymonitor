// Simulator engine. Owns:
//   - a simple battery + sources model that produces realistic ADC counts
//     based on user-controlled sliders
//   - the firmware state (StateOfCharge, EnergyAccumulator, screen index)
//   - timers that mirror the firmware schedule
//   - DOM bindings for the virtual LCD, gauges, buttons, and log stream

import {
    Config, StateOfCharge, EnergyAccumulator, RingBuffer,
    takeSample, backlightFor, SCREENS, csvRow, CSV_HEADER,
} from './firmware.js';

// ---------------------------------------------------------------------------
// Battery + source model
// ---------------------------------------------------------------------------
//
// We model the bank as a coulomb-counted reservoir with a linear OCV ↔ SOC
// relationship between 11.0 V and 13.5 V plus a small "loaded sag" effect:
// voltage drops when load > charge, rises when charging.

const Model = {
    capacityAh: Config.battery.capacityAh,
    socPct: 70.0,                  // start at 70%
    drawA: 0,
    solarA: 0,
    acA: 0,
    carA: 0,

    voltsAtRest() {
        const t = Math.max(0, Math.min(1, this.socPct / 100));
        // OCV roughly 11.5 V empty → 13.0 V full.
        return 11.5 + 1.5 * t;
    },

    voltsUnderLoad() {
        const v0 = this.voltsAtRest();
        const net = this.drawA - (this.solarA + this.acA + this.carA);
        // ~30 mΩ internal — tweak to taste.
        return Math.max(9.0, Math.min(14.6, v0 - net * 0.030));
    },

    step(dtSec) {
        const net = this.drawA - (this.solarA + this.acA + this.carA);
        const deltaAh = (net * dtSec) / 3600;
        this.socPct = Math.max(0, Math.min(100,
            this.socPct - (deltaAh / this.capacityAh) * 100));
    },
};

// ---------------------------------------------------------------------------
// Build ADC counts that the firmware would see, given the model's currents.
// Inverse of CurrentMath.h: amps → ADC count via the same map.
// ---------------------------------------------------------------------------
function ampsToAdc(amps, offset) {
    // Reverse adcToCurrent so the simulated front-end produces a reading the
    // firmware will decode back to (approximately) the input current.
    const corrected = amps - offset;
    const mv = corrected * 1000 * Config.cal.acs712VoltsPerAmp;
    // Inverse of mapLinear(adc, 0,1023, -1650,+1650):
    const adc = Math.round(
        ((mv + Config.cal.adcMidpointMv) / (2 * Config.cal.adcMidpointMv)) *
        Config.cal.adcMaxCount
    );
    return Math.max(0, Math.min(Config.cal.adcMaxCount, adc));
}

function voltsToAdc(volts) {
    const adc = Math.round((volts / Config.cal.batteryDividerVmax) *
                           Config.cal.adcMaxCount);
    return Math.max(0, Math.min(Config.cal.adcMaxCount, adc));
}

// ---------------------------------------------------------------------------
// Firmware state — mirrors what setup() creates in main.cpp.
// ---------------------------------------------------------------------------
const soc = new StateOfCharge();
const drawEnergy = new EnergyAccumulator();
const solarEnergy = new EnergyAccumulator();
const voltageHistory = new RingBuffer(60);

// Seed SOC from initial model voltage (mirrors main.cpp::setup()).
soc.seedFromVoltage(Model.voltsAtRest());

let screen = 0;
let lastSample = null;
let lastButtonTs = 0;
let bootMs = performance.now();
let logBytes = 0;
let logName = '00000000.CSV';
let timeAccel = 1;

// ---------------------------------------------------------------------------
// DOM
// ---------------------------------------------------------------------------
const $ = (s) => document.querySelector(s);

const els = {
    lcdTop: $('#lcd-top'),
    lcdBot: $('#lcd-bot'),
    lcd: $('#lcd'),
    screenName: $('#screen-name'),
    log: $('#log'),

    sliderSolar: $('#slider-solar'),
    sliderAc: $('#slider-ac'),
    sliderCar: $('#slider-car'),
    sliderDraw: $('#slider-draw'),
    sliderAccel: $('#slider-accel'),

    valSolar: $('#val-solar'),
    valAc: $('#val-ac'),
    valCar: $('#val-car'),
    valDraw: $('#val-draw'),
    valAccel: $('#val-accel'),

    gaugeV: $('#gauge-v'),
    gaugeSoc: $('#gauge-soc'),
    gaugeNet: $('#gauge-net'),
    gaugePower: $('#gauge-power'),
    gaugeEnergy: $('#gauge-energy'),
    gaugeSolar: $('#gauge-solar'),
};

// Bind sliders to model.
function bindSlider(slider, valLabel, key, unit) {
    const apply = () => {
        Model[key] = parseFloat(slider.value);
        valLabel.textContent = `${Model[key].toFixed(1)} ${unit}`;
    };
    slider.addEventListener('input', apply);
    apply();
}
bindSlider(els.sliderSolar, els.valSolar, 'solarA', 'A');
bindSlider(els.sliderAc, els.valAc, 'acA', 'A');
bindSlider(els.sliderCar, els.valCar, 'carA', 'A');
bindSlider(els.sliderDraw, els.valDraw, 'drawA', 'A');

els.sliderAccel.addEventListener('input', () => {
    timeAccel = parseFloat(els.sliderAccel.value);
    els.valAccel.textContent = `${timeAccel.toFixed(0)}×`;
});
els.valAccel.textContent = `${timeAccel.toFixed(0)}×`;

// Buttons.
document.querySelectorAll('[data-button]').forEach(btn => {
    btn.addEventListener('click', () => pressButton(btn.dataset.button));
});

document.addEventListener('keydown', (e) => {
    const map = { ArrowLeft: 'left', ArrowRight: 'right',
                  ArrowUp: 'up', ArrowDown: 'down', Enter: 'select' };
    const b = map[e.key];
    if (b) { pressButton(b); e.preventDefault(); }
});

function pressButton(b) {
    const t = performance.now();
    if (t - lastButtonTs < Config.timing.buttonDebounceMs) return;
    lastButtonTs = t;
    if (b === 'left')  screen = (screen + SCREENS.length - 1) % SCREENS.length;
    if (b === 'right') screen = (screen + 1) % SCREENS.length;
    if (b === 'select') {
        // Force a log rotation, mirroring a planned firmware menu action.
        logName = nowFileName();
        logBytes = 0;
        appendLog(`-- rotated to ${logName} --`);
    }
}

// ---------------------------------------------------------------------------
// CSV log buffer in the right panel
// ---------------------------------------------------------------------------
function appendLog(line) {
    els.log.textContent += line + '\n';
    els.log.scrollTop = els.log.scrollHeight;
    // Bound the buffer so the page stays responsive.
    if (els.log.textContent.length > 64 * 1024) {
        els.log.textContent = els.log.textContent.slice(-32 * 1024);
    }
}
appendLog(CSV_HEADER);

function nowFileName() {
    const d = new Date();
    const pad = (n) => String(n).padStart(2, '0');
    return `${pad(d.getFullYear() % 100)}${pad(d.getMonth()+1)}${pad(d.getDate())}${pad(d.getHours())}.CSV`;
}
logName = nowFileName();

// ---------------------------------------------------------------------------
// Backlight colour application
// ---------------------------------------------------------------------------
function applyBacklight(colour) {
    const map = {
        green:  '#7be07b',
        blue:   '#7ec8ff',
        yellow: '#ffe066',
        red:    '#ff7373',
        white:  '#f4f4f4',
        off:    '#1a1a1a',
    };
    els.lcd.style.backgroundColor = map[colour] || map.white;
}

// ---------------------------------------------------------------------------
// Gauge formatters
// ---------------------------------------------------------------------------
function updateGauges(s) {
    els.gaugeV.textContent      = `${s.batteryVolts.toFixed(2)} V`;
    els.gaugeSoc.textContent    = `${s.socPct.toFixed(1)} %`;
    els.gaugeNet.textContent    = `${s.netA >= 0 ? '+' : ''}${s.netA.toFixed(2)} A`;
    els.gaugeNet.style.color    = s.netA >= 0 ? '#ff9b9b' : '#9bff9b';
    els.gaugePower.textContent  = `${s.drawW.toFixed(1)} W`;
    els.gaugeEnergy.textContent = `${s.energyWh.toFixed(1)} Wh`;
    els.gaugeSolar.textContent  = `${s.solarW.toFixed(1)} W`;
}

// ---------------------------------------------------------------------------
// Main loops
// ---------------------------------------------------------------------------
const FW_VERSION = '0.2.0';

function runSample() {
    const dt = Config.timing.measurementPeriodMs / 1000 * timeAccel;
    Model.step(dt);

    const adc = {
        battery: voltsToAdc(Model.voltsUnderLoad()),
        solar:   ampsToAdc(-Model.solarA, Config.cal.solarOffsetA),
        ac:      ampsToAdc(-Model.acA,    Config.cal.acOffsetA),
        car:     ampsToAdc(-Model.carA,   Config.cal.carOffsetA),
        draw:    ampsToAdc( Model.drawA,  Config.cal.drawOffsetA),
    };

    lastSample = takeSample(adc, soc, drawEnergy, solarEnergy);
    voltageHistory.push(lastSample.batteryVolts);
    updateGauges(lastSample);

    const ts = new Date().toISOString().slice(0, 19);
    const row = csvRow(ts, lastSample);
    appendLog(row);
    logBytes += row.length + 1;
    if (logBytes >= Config.logger.maxLogBytes) {
        logName = nowFileName();
        logBytes = 0;
        appendLog(`-- rotated to ${logName} --`);
    }
}

function runDisplay() {
    if (!lastSample) return;
    const uptimeSec = Math.floor((performance.now() - bootMs) / 1000 * timeAccel);
    const ctx = {
        history: voltageHistory,
        logName,
        logBytes,
        uptimeSec,
        fwVersion: FW_VERSION,
    };
    const { top, bottom } = SCREENS[screen].render(lastSample, ctx);
    els.lcdTop.textContent = top;
    els.lcdBot.textContent = bottom;
    els.screenName.textContent = `${screen + 1}/${SCREENS.length} · ${SCREENS[screen].name}`;
    applyBacklight(backlightFor(lastSample));
}

// One immediate sample so the LCD doesn't show blank for 5 s.
runSample();
runDisplay();

// Two periodic timers, scaled by timeAccel.
let sampleTimer = null;
function rescheduleSampleTimer() {
    if (sampleTimer) clearInterval(sampleTimer);
    sampleTimer = setInterval(runSample,
                              Math.max(50, Config.timing.measurementPeriodMs / timeAccel));
}
rescheduleSampleTimer();
els.sliderAccel.addEventListener('input', rescheduleSampleTimer);

setInterval(runDisplay, Config.timing.displayPeriodMs);
