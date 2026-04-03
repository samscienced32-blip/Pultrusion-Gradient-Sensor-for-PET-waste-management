/*
 * =============================================================
 * Prototypic Volumetric Gradient Sensor for PET Pultrusion
 * By Sagar Kumar & J. Monish
 * =============================================================
 *
 * The DRV5055A4 was chosen because its 12.5 mV/mT sensitivity
 * gives enough resolution to detect the small thickness changes
 * in our strips without needing additional amplification. Its
 * quiescent voltage sitting at exactly VCC/2 also means we get
 * a clean bipolar output -- positive deviations for thicker
 * regions, negative for thinner ones -- which maps directly onto
 * our correction direction logic.
 *
 * We're driving a Nema 17 via an A4988/DRV8825 because those
 * were already part of the standard pultrusion system blueprint
 * we based the puller on. Keeping the same motor meant we didn't
 * need to redesign the puller mechanics to accommodate something
 * different, which kept costs and complexity down.
 *
 * Wiring:
 *   Hall Sensor (DRV5055):
 *     VCC  -> 5V
 *     GND  -> GND
 *     OUT  -> A0
 *
 *   Stepper Driver (A4988):
 *     STEP -> D3
 *     DIR  -> D4
 *     EN   -> D5 (active LOW)
 *
 *   Optional NTC thermistor (nozzle temp):
 *     One leg -> A1 + 10kOhm to GND (voltage divider)
 *     Other   -> 5V
 *
 *   Status LED:
 *     Anode -> D13 (built-in), Cathode -> GND
 * =============================================================
 */

// ---- Pin Definitions ----
#define HALL_PIN        A0
#define TEMP_PIN        A1
#define STEP_PIN         3
#define DIR_PIN          4
#define EN_PIN           5
#define LED_PIN         13

// ---- DRV5055A4 Sensor Constants ----
// We're running the sensor at 5V because it gives us the full
// +/-100mT measurement range. At 3.3V the range shrinks and we'd
// lose sensitivity at the extremes -- not ideal when some of our
// degraded bottles produce quite large volumetric peaks.
#define VCC_MV          5000.0

// 12.5 mV/mT is the A4 variant's sensitivity at 5V and 25 degrees C.
// We picked the A4 specifically over the A1 (3.125 mV/mT) because
// our lever arm gives a 1:3 amplification ratio, but the magnet
// displacement is still small enough that we needed the higher
// sensitivity variant to get a usable voltage swing.
#define SENSOR_SENSITIVITY 12.5

// The datasheet quotes 0.12%/degC as the typical STC. We apply
// this correction because the sensor sits close to the heated
// base (60 degC roller nearby), so ambient temperature around the
// sensor drifts during operation and would otherwise introduce
// a false offset into our readings.
#define STC             0.0012

// VQ sits at exactly half of VCC by design. This is why a strip
// at nominal thickness should read ~2500 mV -- the magnet is
// centred relative to the hall sensor, producing no net field bias.
#define VQ_MV           (VCC_MV / 2.0)
#define ADC_RESOLUTION  1024.0

// ---- Pultrusion System Parameters ----
// 2500 mV is our baseline: it's what the sensor outputs when lever 2
// is in its neutral position, meaning the strip underneath is at the
// nominal thickness we expect from an undamaged bottle section.
// You'll need to re-measure this with your specific magnet placement.
#define NOMINAL_VOLTAGE_MV   2500.0

// We set the deadband to 30 mV because that's roughly 3x the
// noise floor we measured with no strip in the sensor. Correcting
// within the noise would just cause the stepper to hunt back and
// forth at nominal thickness, which would actually introduce
// gradients rather than remove them.
#define VOLTAGE_DEADBAND_MV    30.0

// The nominal rate of 800 steps/sec was tuned to hit 0.38 cm/s,
// which our experiments showed gave the best volumetric gradient
// reduction at 235 degC nozzle temperature. Going faster than this
// at nominal didn't give the nozzle enough time to fully melt
// the strip, and slower caused inconsistent melt pooling.
#define NOMINAL_STEP_RATE    800
#define MIN_STEP_RATE        400
#define MAX_STEP_RATE       1400

// The correction gain of 0.35 steps/sec per mV came from our
// regression analysis -- we plotted pull rate change against
// volumetric gradient reduction and found this slope gave the
// fastest correction without causing the system to overshoot
// and swing between over- and under-correction.
#define CORRECTION_GAIN      0.35

// We sample every 50 ms because that gives us a 20 Hz control
// loop, which is fast enough to react before a gradient peak
// travels the distance from the sensor to the nozzle at our
// pull rate. Any slower and the correction would arrive too late.
#define SAMPLE_COUNT         10
#define CONTROL_INTERVAL_MS  50
#define SERIAL_PRINT_MS     500

// We found 200 Hz to be the sweet spot for the Hall sensor
// polling frequency in our experiments. Below ~100 Hz we
// started missing fast gradient transitions in the strips;
// above 500 Hz the ADC noise started dominating the signal.
// 10 oversamples inside each 50 ms window gives us exactly
// the 200 Hz effective sampling rate we wanted.
#define HALL_OVERSAMPLE      10

// ---- Stepper State ----
volatile unsigned long stepInterval_us = 0;
unsigned long lastStepTime_us = 0;
bool stepperEnabled = true;
int stepperDir = HIGH;

// ---- Control State ----
float currentPullRate   = NOMINAL_STEP_RATE;
float measuredVoltageMV = VQ_MV;
float deviationMV       = 0.0;
float ambientTempC      = 25.0;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime   = 0;
bool correctionActive = false;

// =============================================================
// SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== PET Pultrusion Gradient Sensor ==="));
  Serial.println(F("Sagar Kumar & J. Monish"));
  Serial.println(F("--------------------------------------"));

  pinMode(HALL_PIN,  INPUT);
  pinMode(TEMP_PIN,  INPUT);
  pinMode(STEP_PIN,  OUTPUT);
  pinMode(DIR_PIN,   OUTPUT);
  pinMode(EN_PIN,    OUTPUT);
  pinMode(LED_PIN,   OUTPUT);

  // EN is active LOW on the A4988, so pulling it LOW here
  // enables the driver immediately on startup. We keep it
  // enabled throughout -- disabling between corrections would
  // cause the stepper to lose holding torque and let the strip
  // slip, which would corrupt the position timing of our corrections.
  digitalWrite(EN_PIN,  LOW);
  digitalWrite(DIR_PIN, HIGH); // We only ever pull, never reverse
  digitalWrite(STEP_PIN, LOW);

  setPullRate(NOMINAL_STEP_RATE);

  Serial.println(F("System ready. Starting nominal pull rate."));
  Serial.print(F("Nominal rate: "));
  Serial.print(NOMINAL_STEP_RATE);
  Serial.println(F(" steps/sec"));
  Serial.println(F(""));
  Serial.println(F("Time(ms)\tHall(mV)\tDeviation(mV)\tPullRate(sps)\tStatus"));
}

// =============================================================
// MAIN LOOP
// =============================================================
void loop() {
  unsigned long now = millis();

  // The stepper needs to be pulsed on every iteration of loop()
  // rather than inside a delay, because any blocking call here
  // would cause the motor to stall mid-correction. At our pull
  // rates even a 5 ms delay is enough to cause a visible jerk
  // in the filament diameter.
  runStepper();

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = now;

    measuredVoltageMV = readHallMV(HALL_OVERSAMPLE);
    deviationMV       = measuredVoltageMV - NOMINAL_VOLTAGE_MV;

    if (abs(deviationMV) > VOLTAGE_DEADBAND_MV) {
      correctionActive = true;
      applyCorrection(deviationMV);
    } else {
      correctionActive = false;
      // Rather than snapping back to nominal instantly, we ease
      // toward it at 5% per tick. An abrupt rate change here would
      // itself introduce a small volumetric gradient at the nozzle
      // as the strip transitions out of a corrected region.
      currentPullRate += (NOMINAL_STEP_RATE - currentPullRate) * 0.05;
      setPullRate((int)currentPullRate);
    }

    digitalWrite(LED_PIN, correctionActive ? HIGH : LOW);
  }

  if (now - lastPrintTime >= SERIAL_PRINT_MS) {
    lastPrintTime = now;
    printStatus(now);
  }
}

// =============================================================
// HALL EFFECT SENSOR READING
// =============================================================
float readHallMV(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(HALL_PIN);
    // 200 us between samples keeps us at ~5 kHz raw ADC rate,
    // which is well within the DRV5055's 20 kHz bandwidth so we're
    // not losing any real signal -- just averaging out high-frequency
    // electrical noise from the stepper driver switching.
    delayMicroseconds(200);
  }
  float avgADC = (float)sum / samples;
  float rawMV  = (avgADC / ADC_RESOLUTION) * VCC_MV;

  // We normalise back to 25 degC equivalent here so that the rest of
  // the code can treat NOMINAL_VOLTAGE_MV as a fixed reference.
  // Without this, a warm sensor would read slightly differently than
  // a cold one and our deadband would no longer be centred correctly.
  float stcFactor   = 1.0 + STC * (ambientTempC - 25.0);
  float correctedMV = VQ_MV + (rawMV - VQ_MV) / stcFactor;

  return correctedMV;
}

// =============================================================
// CORRECTIONAL MODEL
// =============================================================
void applyCorrection(float deviationMV) {
  // A positive deviation means the strip is thicker than nominal,
  // so more PET is arriving at the nozzle than expected. We speed
  // up the puller to stretch that section thin before it reaches
  // the nozzle, reducing the volume back toward nominal.
  //
  // A negative deviation means the strip is thinner, so we slow
  // down to let the nozzle compress that section and bring the
  // volume back up.
  //
  // This is the core of our correctional model -- the pull rate is
  // the only variable we can change in real time, so it carries
  // the entire burden of volumetric correction.
  float correction = CORRECTION_GAIN * deviationMV;
  currentPullRate  = NOMINAL_STEP_RATE + correction;
  currentPullRate  = constrain(currentPullRate, MIN_STEP_RATE, MAX_STEP_RATE);

  setPullRate((int)currentPullRate);
}

// =============================================================
// STEPPER MOTOR CONTROL
// =============================================================
void setPullRate(int stepsPerSec) {
  if (stepsPerSec <= 0) {
    stepInterval_us = 0;
    return;
  }
  // Converting to microseconds per step here because micros()
  // gives us the resolution we need to hold a stable rate.
  // If we worked in milliseconds we'd only be able to represent
  // rates in 1000-step/sec increments, which is too coarse for
  // the fine corrections our regression model calls for.
  stepInterval_us = 1000000UL / (unsigned long)stepsPerSec;
}

void runStepper() {
  if (stepInterval_us == 0) return;

  unsigned long now_us = micros();
  if (now_us - lastStepTime_us >= stepInterval_us) {
    lastStepTime_us = now_us;
    digitalWrite(STEP_PIN, HIGH);
    // 2 us is the minimum STEP pulse width the A4988 datasheet
    // requires. We can't go shorter or the driver may miss the pulse.
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);
  }
}

// =============================================================
// AMBIENT TEMPERATURE READING (NTC Thermistor)
// =============================================================
float readAmbientTempC() {
  // We used the B-parameter equation rather than the full
  // Steinhart-Hart because it only needs two datasheet values
  // (B and R at 25 degC) instead of three calibration coefficients.
  // The accuracy is sufficient for our STC correction -- we only
  // need temperature to within a degree or two, not clinical precision.
  const float B_COEFF   = 3950.0;
  const float R_NOMINAL = 10000.0;
  const float R_SERIES  = 10000.0;

  int raw = analogRead(TEMP_PIN);
  if (raw <= 0 || raw >= 1023) return 25.0;

  float vRatio     = (float)raw / ADC_RESOLUTION;
  float resistance = R_SERIES * vRatio / (1.0 - vRatio);

  float steinhart  = resistance / R_NOMINAL;
  steinhart        = log(steinhart);
  steinhart       /= B_COEFF;
  steinhart       += 1.0 / (25.0 + 273.15);
  float tempK      = 1.0 / steinhart;
  return tempK - 273.15;
}

// =============================================================
// SERIAL STATUS PRINT
// =============================================================
void printStatus(unsigned long now) {
  // We update ambient temp here rather than in the control loop
  // because temperature changes slowly -- reading it every 500 ms
  // is more than frequent enough, and keeping it out of the 50 ms
  // loop means we're not wasting ADC time on a slowly-changing
  // variable when we need that time for the Hall sensor.
  ambientTempC = readAmbientTempC();

  Serial.print(now);
  Serial.print(F("\t"));
  Serial.print(measuredVoltageMV, 1);
  Serial.print(F("\t\t"));
  Serial.print(deviationMV, 1);
  Serial.print(F("\t\t"));
  Serial.print((int)currentPullRate);
  Serial.print(F("\t\t"));
  if (correctionActive) {
    if (deviationMV > 0) Serial.println(F("STRETCHING (high volume)"));
    else                 Serial.println(F("COMPRESSING (low volume)"));
  } else {
    Serial.println(F("NOMINAL"));
  }
}

// =============================================================
// CALIBRATION NOTES
// =============================================================
/*
 * NOMINAL_VOLTAGE_MV:
 *   Feed a strip of known-good nominal thickness through and read
 *   the Hall output directly from Serial. That value becomes your
 *   new NOMINAL_VOLTAGE_MV. Don't rely on the theoretical 2500 mV
 *   because magnet placement and lever geometry shift it in practice.
 *
 * VOLTAGE_DEADBAND_MV:
 *   Run the sensor with no strip loaded and note how much the
 *   reading fluctuates. Set the deadband to about 3x that spread.
 *
 * CORRECTION_GAIN:
 *   Start at 0.1 and increase slowly while watching Serial Plotter.
 *   You want the pull rate to settle at a corrected value without
 *   oscillating. If it hunts back and forth, the gain is too high.
 *
 * NOMINAL_STEP_RATE:
 *   Measure actual filament output speed with a ruler and stopwatch.
 *   Adjust until you hit 0.38 cm/s -- the rate our experiments showed
 *   gave the best results at 235 degC nozzle temperature.
 *
 * Serial Plotter:
 *   Tools > Serial Plotter at 115200 baud lets you watch Hall voltage,
 *   deviation, and pull rate live. It's the fastest way to verify
 *   your calibration values are working correctly.
 */
