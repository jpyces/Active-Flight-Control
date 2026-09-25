#ifdef GNC_HARDWARE_BUILD

#include <Arduino.h>
#include <cmath>
#include "LittleFS.h"

#include "config.h"
#include "Altimeter.h"
#include "Imu.h"
#include "Gnss.h"
#include "Magnetometer.h" // matches lowercase convention of the other sensor headers above —
                          // rename Magnetometer.h/.cpp to magnetometer.h/.cpp if they aren't already

using namespace gnc;

// Globals
LittleFS_QSPI myfs; // for flash unit test
// ADD VERSIONING HERE

// Objects/Structs
Altimeter alti;
Imu imu;
GNSS gnss;
Magnetometer mag;
GnssData gnss_data;

// Sensor global data
// // Altimeter
float seaLevelPressurehPa;
float seaLevelPressurePa;
float rawPressurePa;
float pressurehPa;

// Cached "last serviced" values — populated by serviceSensors() at SENSOR_TICK_MS cadence,
// only ever *read* by printSensorDebug(). Printing never re-reads a sensor or calls
// checkHealth() itself, so the 1Hz print cadence can't perturb the 50Hz service/debounce
// cadence, and vice versa.
static SensorStatus altiStatus = SensorStatus::UNINITIALIZED;
static SensorStatus imuStatus = SensorStatus::UNINITIALIZED;
static SensorStatus gnssStatus = SensorStatus::UNINITIALIZED;
static SensorStatus magStatus = SensorStatus::UNINITIALIZED;
static bool gnssAvailable = false;

static float cachedTemperatureC = 0.0F;
static float cachedAltitudeM = 0.0F;

static Imu::RawSample cachedRaw{};
static Imu::MotionSample cachedMotion{};
static bool cachedAccelNearLimit = false;
static bool cachedGyroNearLimit = false;

static Magnetometer::MagSample cachedMag{};
static bool cachedFieldNearLimit = false;

static GNSS::HealthDiagnostics cachedGnssDiag{};

// General Servicing
// // Timing State for sensors and loops
unsigned long loopCount = 0;
unsigned long loopCountAtLastPrint = 0;

unsigned long lastSensorTick = 0;
unsigned long lastPrintTime = 0;
constexpr unsigned long SENSOR_TICK_MS = 20;      // ~50Hz — comfortably faster than GNSS's 10Hz output,
                                                  // fast enough to drain GNSS_SERIAL before it backs up, slow enough that threshold tuning stays sane
constexpr unsigned long PRINT_INTERVAL_MS = 1000; // debug cadence, independent of sensor servicing

// // LED heartbeat — a simple "is the board alive" visual, independent of sensor/print timing
unsigned long lastLedToggle = 0;
bool ledState = false;
constexpr unsigned long LED_TOGGLE_INTERVAL_MS = 1000; // toggles once a second -> on/off every 1s

bool showWakeUpAscii = true;

void wakeUpAscii()
{

    Serial.println(F(""));
    Serial.println(F("        \\   |   /                     \\   |   /"));
    Serial.println(F("         \\  Y  /                       \\  Y  /"));
    Serial.println(F("          \\ | /                         \\ | /"));
    Serial.println(F("           \\|/                           \\|/"));
    Serial.println(F("        ----O----                     ----O----"));
    Serial.println(F("           /|\\                           /|\\"));
    Serial.println(F("          / | \\                         / | \\"));
    Serial.println(F("         /  |  \\                       /  |  \\"));
    Serial.println(F("        [=======]                     [=======]"));
    Serial.println(F("           ||                             ||"));
    Serial.println(F("           ||_____                   _____||"));
    Serial.println(F("                  \\                 /"));
    Serial.println(F("                   \\_______________/"));
    Serial.println(F("                   |               |"));
    Serial.println(F("                   |               |"));
    Serial.println(F("                   |               |"));
    Serial.println(F("                   |               |"));
    Serial.println(F("                   |               |"));
    Serial.println(F("                   |_______________|"));
    Serial.println(F("                   /                \\"));
    Serial.println(F("                  /                  \\"));
    Serial.println(F("           ||_____|                 |_____||"));
    Serial.println(F("           ||                             ||"));
    Serial.println(F("        [=======]                     [=======]"));
    Serial.println(F("         \\  |  /                       \\  |  /"));
    Serial.println(F("          \\ | /                         \\ | /"));
    Serial.println(F("           \\|/                           \\|/"));
    Serial.println(F("        ----O----                     ----O----"));
    Serial.println(F("           /|\\                           /|\\"));
    Serial.println(F("          / Y \\                         / Y \\"));
    Serial.println(F("         /  |  \\                       /  |  \\"));
    Serial.println(F("        /   |   \\                     /   |   \\"));
    Serial.println(F(""));
    Serial.println(F("   ====================================================="));
    Serial.println(F("   >>>            INITIALIZING SYSTEM...            <<<"));
    Serial.println(F("   ====================================================="));
    Serial.println(F(""));

    delay(300);
}

void wakeUp()
{
    // Setting Radio mode pins to 0 to boot into normal mode - connection to software means it is configurable in the future
    pinMode(22, OUTPUT);
    digitalWrite(22, LOW); // M0
    pinMode(23, OUTPUT);
    digitalWrite(23, LOW); // M1

    Serial.begin(DEBUG_BAUD);
    GNSS_SERIAL.begin(GNSS_BAUD);
    RADIO_SERIAL.begin(RADIO_BAUD);

    uint32_t bootTime = millis();
    uint8_t bootSecondCount = 5;

    // Wait for serial debug if not initiated
    while (!Serial)
    {
    }

    // General wake-up window
    while ((millis() - bootTime) < BOOT_SEQ_DELAY_MS)
    {
        // Give the USB serial monitor a moment to attach after upload/reset.
        Serial.print(bootSecondCount);
        delay(200);
        Serial.print(".");
        delay(200);
        Serial.print(".");
        delay(200);
        Serial.print(". ");
        bootSecondCount--;
        delay(400);
    }

    // Fun
    if (showWakeUpAscii)
    {
        wakeUpAscii();
    }

    // Activate I2C lines - some libraries do it, but preemptive activation ensures that they work
    Wire.begin();
    Wire1.begin();
    Wire2.begin();

    // Heartbeat LED (Teensy 4.1 onboard, pin 13 / LED_BUILTIN)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void sensorStartup()
{
    // Altimeter
    Serial.println(alti.begin() ? "Altimeter Detected and Started" : "Failed");

    // Assumed to be on ground during setup
    Serial.println("\nAltimeter Pressure Reading:");
    seaLevelPressurePa = alti.getPressure();
    seaLevelPressurehPa = seaLevelPressurePa / 100;
    Serial.println(seaLevelPressurehPa);

    // IMU
    Serial.println("\nIMU:");
    Serial.println(imu.begin() ? "Detected and Started" : "Failed");
    Serial.println(imu.configureForFlight() ? "Configured for flight" : "Configuration failed");

    // GNSS runs at 10Hz
    Serial.println("\nGNSS:");
    gnssAvailable = gnss.begin();
    if (!gnssAvailable)
    {
        Serial.println("GNSS UNAVAILABLE");
        gnssStatus = gnss.getStatus();
    }

    // Magnetometer
    Serial.println("\nMagnetometer:");
    Serial.println(mag.begin() ? "Detected and Started" : "Failed");
    Serial.println(mag.configureForFlight() ? "Configured for flight" : "Configuration failed");

    // Seed the caches so the first print (up to PRINT_INTERVAL_MS after boot) has something
    // real to show instead of zero-initialized placeholders.
    altiStatus = alti.checkHealth();
    imuStatus = imu.checkHealth();

    cachedTemperatureC = alti.getTemperature();
    rawPressurePa = alti.getPressure();
    pressurehPa = rawPressurePa / 100;
    cachedAltitudeM = alti.getAltitude(seaLevelPressurehPa);

    cachedRaw = imu.getRawSample();
    cachedMotion = imu.getMotionSample();
    cachedAccelNearLimit = imu.isAccelNearLimit();
    cachedGyroNearLimit = imu.isGyroNearLimit();

    cachedMag = mag.getMagSample();
    magStatus = mag.getStatus();
    cachedFieldNearLimit = mag.isFieldNearLimit(cachedMag);

    if (gnssAvailable)
    {
        gnss_data = gnss.getData(gnss_data);
        gnssStatus = gnss.getStatus();
        cachedGnssDiag = gnss.getHealthDiagnostics();
    }
}

// Runs at SENSOR_TICK_MS (~50Hz): the only place sensors are actually read or health-checked.
// Nothing here touches Serial — keeps this tick's timing independent of how long printing takes.
void serviceSensors()
{
    altiStatus = alti.checkHealth();
    cachedTemperatureC = alti.getTemperature();
    rawPressurePa = alti.getPressure();
    pressurehPa = rawPressurePa / 100;
    cachedAltitudeM = alti.getAltitude(seaLevelPressurehPa);

    imuStatus = imu.checkHealth();
    cachedRaw = imu.getRawSample();
    cachedMotion = imu.getMotionSample();
    cachedAccelNearLimit = imu.isAccelNearLimit();
    cachedGyroNearLimit = imu.isGyroNearLimit();

    cachedMag = mag.getMagSample();
    magStatus = mag.getStatus();
    cachedFieldNearLimit = mag.isFieldNearLimit(cachedMag);

    // GNSS parsing needs frequent servicing once startup succeeds, but a failed begin()
    // leaves the library without a fully initialized receiver contract to service.
    if (gnssAvailable)
    {
        gnss_data = gnss.getData(gnss_data);
        gnssStatus = gnss.getStatus();
        cachedGnssDiag = gnss.getHealthDiagnostics();
    }

    loopCount++;
}

// Runs at PRINT_INTERVAL_MS (~1Hz): prints whatever serviceSensors() last cached. Never reads
// a sensor directly, so the human-readable cadence can be slow without slowing sensor servicing.
void printSensorDebug()
{
    // loop() advances lastPrintTime on a fixed PRINT_INTERVAL_MS cadence (not "now"), so the
    // elapsed time since the previous print is PRINT_INTERVAL_MS as long as nothing stalled —
    // dividing the tick delta by that gives a cheap sanity check on the real achieved service rate.
    unsigned long loopsSinceLastPrint = loopCount - loopCountAtLastPrint;
    float achievedServiceHz = (1000.0F * static_cast<float>(loopsSinceLastPrint)) / static_cast<float>(PRINT_INTERVAL_MS);
    loopCountAtLastPrint = loopCount;

    Serial.print("Loop: ");
    Serial.print(loopCount);
    Serial.print("  (service rate ~");
    Serial.print(achievedServiceHz, 1);
    Serial.println(" Hz)");

    Serial.println("\n1. Altimeter Health Check:");
    printStatus(altiStatus);

    Serial.print("\nAltimeter Temperature Reading: ");
    Serial.println(cachedTemperatureC);

    Serial.print("\nSealevel Pressure Reading (hPa): ");
    Serial.println(seaLevelPressurehPa);
    Serial.print("Pressure in Pa: ");
    Serial.print(rawPressurePa);
    Serial.print("\nPressure in hPa: ");
    Serial.print(pressurehPa);

    Serial.print("\nAltimeter Altitude Reading: ");
    Serial.println(cachedAltitudeM);

    Serial.println("\n---\n");

    // ---- IMU ----

    Serial.println("2. IMU Health Check:");
    printStatus(imuStatus);

    Serial.println("\nIMU Raw Sample:");
    Serial.print("  accel raw: ");
    Serial.print(cachedRaw.accelX);
    Serial.print(", ");
    Serial.print(cachedRaw.accelY);
    Serial.print(", ");
    Serial.println(cachedRaw.accelZ);
    Serial.print("  gyro raw:  ");
    Serial.print(cachedRaw.gyroX);
    Serial.print(", ");
    Serial.print(cachedRaw.gyroY);
    Serial.print(", ");
    Serial.println(cachedRaw.gyroZ);
    Serial.print("  temp raw:  ");
    Serial.println(cachedRaw.temperature);

    Serial.println("\nIMU Motion Sample:");
    Serial.print("  accel (g):   x=");
    Serial.print(cachedMotion.accelG.x, 3);
    Serial.print(" y=");
    Serial.print(cachedMotion.accelG.y, 3);
    Serial.print(" z=");
    Serial.println(cachedMotion.accelG.z, 3);
    Serial.print("  gyro (dps):  x=");
    Serial.print(cachedMotion.gyroDps.x, 3);
    Serial.print(" y=");
    Serial.print(cachedMotion.gyroDps.y, 3);
    Serial.print(" z=");
    Serial.println(cachedMotion.gyroDps.z, 3);
    Serial.print("  accel mag (g): ");
    Serial.println(cachedMotion.accelMagnitudeG, 3);
    Serial.print("  gyro mag (dps): ");
    Serial.println(cachedMotion.gyroMagnitudeDps, 3);
    Serial.print("  temp (C): ");
    Serial.println(cachedMotion.temperatureC, 2);

    Serial.print("  accel near limit: ");
    Serial.println(cachedAccelNearLimit ? "YES" : "no");
    Serial.print("  gyro near limit:  ");
    Serial.println(cachedGyroNearLimit ? "YES" : "no");

    Serial.println("\n---\n");

    // ---- Magnetometer ----

    Serial.println("3. Magnetometer Health Check:");
    printStatus(magStatus);

    Serial.println("\nMagnetometer Field Sample:");
    Serial.print("  field (uT):  x=");
    Serial.print(cachedMag.fieldUT.x, 2);
    Serial.print(" y=");
    Serial.print(cachedMag.fieldUT.y, 2);
    Serial.print(" z=");
    Serial.println(cachedMag.fieldUT.z, 2);
    Serial.print("  field mag (uT): ");
    Serial.println(cachedMag.fieldMagnitudeUT, 2);

    Serial.print("  field near limit: ");
    Serial.println(cachedFieldNearLimit ? "YES" : "no");

    Serial.println("\n---\n");

    Serial.println("4. GNSS Status: ");
    printStatus(gnssStatus);

    Serial.print("  ever had a fix: ");
    Serial.println(cachedGnssDiag.hasEverFixed ? "yes" : "no");
    Serial.print("  ms since last NAV-PVT: ");
    Serial.println(cachedGnssDiag.msSinceLastPvt);
    Serial.print("  consecutive good fixes: ");
    Serial.println(cachedGnssDiag.consecutiveSuccesses);
    Serial.print("  consecutive bad-fix messages: ");
    Serial.println(cachedGnssDiag.consecutiveFailures);

    Serial.println(); // formatting
    gnss_data.print();

    Serial.println("\n-------------------------------------------\n");
}

void setup()
{
    wakeUp();

    Serial.println("\n\n--------------\nEntered Setup!\n--------------\n");

    sensorStartup();

    Serial.println("\n-----------\nSETUP ENDED\n-----------\n\n-------------------------------------------\n\n");

    lastSensorTick = millis();
    lastPrintTime = millis();
    lastLedToggle = millis();
}

void loop()
{
    unsigned long now = millis();

    if (now - lastSensorTick >= SENSOR_TICK_MS)
    {
        lastSensorTick += SENSOR_TICK_MS; // fixed-cadence accumulation, not "now" — avoids
                                          // cumulative drift if a tick occasionally runs long
        serviceSensors();
    }

    if (now - lastPrintTime >= PRINT_INTERVAL_MS)
    {
        lastPrintTime += PRINT_INTERVAL_MS;
        printSensorDebug();
    }

    if (now - lastLedToggle >= LED_TOGGLE_INTERVAL_MS)
    {
        lastLedToggle += LED_TOGGLE_INTERVAL_MS;
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    }
}

/*
CODE SCRAPS
loop:
    // GNSS_SERIAL.write(0x55); // arbitrary test byte
    // delay(500);
    // while (GNSS_SERIAL.available())
    // {
    //     Serial.println(GNSS_SERIAL.read(), HEX);
    // }
    // while (GNSS_SERIAL.available())
    // {
    //     uint8_t b = GNSS_SERIAL.read();
    //     if (b < 0x10)
    //         Serial.print("0");
    //     Serial.print(b, HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();
    // Serial3.write(0x55); // arbitrary test byte
    // delay(500);
    // while (Serial3.available())
    // {
    //     Serial.println(Serial3.read(), HEX);
    // }

*/
#endif