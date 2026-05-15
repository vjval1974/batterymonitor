// Cross-check that the JS firmware port matches the C++ math.
// Runs with vanilla Node: `node sim/test_firmware.mjs`.

import {
    adcToCurrent, adcToBatteryVolts, voltageToSoc,
    StateOfCharge, EnergyAccumulator, RingBuffer, takeSample, SCREENS,
} from './firmware.js';

let failures = 0;
function check(name, cond) {
    if (cond) {
        console.log(`  ok  ${name}`);
    } else {
        console.error(`  FAIL ${name}`);
        ++failures;
    }
}
function near(a, b, eps = 0.05) { return Math.abs(a - b) <= eps; }

console.log('CurrentMath:');
check('adcToCurrent(512,0) clamped to 0', adcToCurrent(512, 0) === 0);
check('adcToCurrent(1023,0) ≈ +25 A', near(adcToCurrent(1023, 0), 25.0, 0.3));
check('adcToCurrent(0,0)   ≈ -25 A', near(adcToCurrent(0, 0), -25.0, 0.3));
check('adcToBatteryVolts(1023) ≈ 15.88', near(adcToBatteryVolts(1023), 15.88, 0.02));

console.log('SocLookup:');
check('voltageToSoc(10)  == 0', voltageToSoc(10) === 0);
check('voltageToSoc(13)  == 100', voltageToSoc(13) === 100);
check('voltageToSoc(12.13) ≈ 55', near(voltageToSoc(12.13), 55, 1.0));

console.log('StateOfCharge:');
{
    const soc = new StateOfCharge(100);
    soc.socPct = 80;
    soc.update(10, 3600);
    check('discharge 10 A·h from 80% → 70%', near(soc.socPct, 70, 0.05));
}
{
    const soc = new StateOfCharge(100);
    soc.socPct = 75;
    for (let i = 0; i < 6; i++) soc.maybeReanchor(13.6, 0, 60);
    check('float-voltage reanchor → 100%', soc.socPct === 100);
}

console.log('EnergyAccumulator:');
{
    const e = new EnergyAccumulator();
    e.accumulate(120, 3600);
    e.accumulate(60, 1800);
    check('120 W·h + 30 W·h → 150 W·h', near(e.wh, 150, 0.001));
}

console.log('RingBuffer:');
{
    const rb = new RingBuffer(4);
    [1, 2, 3, 4, 5].forEach(v => rb.push(v));
    check('after wrap: min=2', rb.min() === 2);
    check('after wrap: max=5', rb.max() === 5);
    check('after wrap: mean=3.5', rb.mean() === 3.5);
}

console.log('Screen layouts (16 chars each):');
{
    const soc = new StateOfCharge();
    const drawE = new EnergyAccumulator();
    const solarE = new EnergyAccumulator();
    const s = takeSample({ battery: 800, solar: 400, ac: 512, car: 512, draw: 600 },
                         soc, drawE, solarE);
    const ctx = {
        history: new RingBuffer(8), logName: '24011500.CSV', logBytes: 1234,
        uptimeSec: 90061, fwVersion: '0.2.0',
    };
    ctx.history.push(s.batteryVolts);
    for (let i = 0; i < SCREENS.length; ++i) {
        const r = SCREENS[i].render(s, ctx);
        check(`screen ${i} (${SCREENS[i].name}) top is 16 chars`,  r.top.length === 16);
        check(`screen ${i} (${SCREENS[i].name}) bot is 16 chars`,  r.bottom.length === 16);
    }
}

if (failures === 0) {
    console.log('\nOK — all simulator math matches firmware.');
    process.exit(0);
} else {
    console.error(`\n${failures} failure(s)`);
    process.exit(1);
}
