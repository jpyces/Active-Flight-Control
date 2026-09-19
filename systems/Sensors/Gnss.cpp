#include "Gnss.h"
#include <cmath>
#include "config.h"

using namespace gnc;

GNSS::GNSS()
    : gnss(), gnc_DEBUG(false), status(SensorStatus::UNINITIALIZED),
      consecutiveSuccesses(0), consecutiveFailures(0), lastPvtMillis(0),
      lastTimeOfWeekMs(0), hasTimeOfWeek(false), hasEverFixed(false),
      firstFixDeadlineMillis(0) {
      };

bool GNSS::begin()
{
    GNSS_SERIAL.begin(GNSS_BAUD);
    Serial.print("Debugging status: ");
    Serial.println(gnc_DEBUG);
    if (gnc_DEBUG)
        gnss.enableDebugging(Serial);

    static constexpr unsigned long BEGIN_TIMEOUT_MS = 30000;
    Serial.println("Waiting for GNSS to begin (30 second timeout max)...");
    bool ok = gnss.begin(GNSS_SERIAL);

    unsigned long start = millis();
    while (!ok && (millis() - start < BEGIN_TIMEOUT_MS))
    {
        Serial.println("GNSS NOT ACTIVE, retrying...");
        delay(1500); // give the previous begin()'s internal handshake time to fully resolve
        ok = gnss.begin(GNSS_SERIAL);
    }

    if (!ok)
    {
        Serial.println("30 seconds elapsed, GNSS failed to start.");
        status = SensorStatus::FAILED;
        return false;
    }

    Serial.println("GNSS Active!");
    if (!gnss.setUART1Output(COM_TYPE_UBX))
        Serial.println("WARNING: setUART1Output failed!");
    if (!gnss.setUART1Input(COM_TYPE_UBX))
        Serial.println("WARNING: setUART1Input failed!");

    Serial.print("New Valset to GNSS module: ");
    Serial.println(gnss.newCfgValset());
    gnss.addCfgValset(UBLOX_CFG_RATE_MEAS, 100);
    gnss.addCfgValset(UBLOX_CFG_RATE_NAV, 1);
    if (!gnss.sendCfgValset())
    {
        Serial.println("FATAL: sendCfgValset failed! Rate config not applied — aborting begin().");
        status = SensorStatus::FAILED;
        return false;
    }

    if (!gnss.setAutoPVT(true))
    {
        Serial.println("FATAL: setAutoPVT failed! PVT stream not enabled — aborting begin().");
        status = SensorStatus::FAILED;
        return false;
    }

    // Start the silence clock now, not at some earlier point during the (up to 30s) begin()
    // retry loop above — otherwise checkHealth() could see a false "2.5s of silence" on its
    // very first call before the module has even had a chance to send its first NAV-PVT.
    lastPvtMillis = millis();
    lastTimeOfWeekMs = 0;
    hasTimeOfWeek = false;

    // Give the module its one-time grace window to acquire a first fix (see header comment)
    // before "no good fix yet" starts counting toward DEGRADED.
    hasEverFixed = false;
    firstFixDeadlineMillis = millis() + FIRST_FIX_GRACE_MS;
    status = SensorStatus::NOMINAL;

    return true;
}

SensorStatus GNSS::checkHealth()
{
    // getData() is the sole hardware-touching path — see its doc comment and the header's
    // comment on checkHealth() for why duplicating a second independent read here (as this
    // used to do, via its own gnss.getPVT() call) is exactly the bug that caused false FAILED
    // verdicts despite the receiver working fine.
    GnssData scratch{};
    getData(scratch);
    return status;
}

void GNSS::recordFreshPvt(bool fixOk, unsigned long now)
{
    lastPvtMillis = now;

    if (fixOk)
    {
        hasEverFixed = true;
        consecutiveFailures = 0;
        consecutiveSuccesses++;
        if (consecutiveSuccesses >= FULL_RECOVERY_THRESHOLD)
            status = SensorStatus::NOMINAL;
        else if (consecutiveSuccesses >= RECOVERY_THRESHOLD)
            status = SensorStatus::DEGRADED;
    }
    else if (!hasEverFixed && now < firstFixDeadlineMillis)
    {
        // Still inside the first-fix grace window and never had a fix yet. This is ordinary
        // cold-start acquisition, not a fault.
    }
    else
    {
        consecutiveSuccesses = 0;
        consecutiveFailures++;
        if (status != SensorStatus::FAILED && consecutiveFailures >= DEGRADE_THRESHOLD)
            status = SensorStatus::DEGRADED;
    }
}

void GNSS::recordPvtSilence(unsigned long now)
{
    // No new NAV-PVT since the last check is expected on most service ticks at 10Hz output.
    // Only genuine staleness, measured in real elapsed time, escalates.
    unsigned long staleMs = now - lastPvtMillis;

    if (staleMs >= SILENCE_FAILURE_MS)
    {
        status = SensorStatus::FAILED;
        consecutiveSuccesses = 0;
        consecutiveFailures = 0;
    }
    else if (staleMs >= SILENCE_DEGRADE_MS && status != SensorStatus::FAILED)
    {
        status = SensorStatus::DEGRADED;
    }
}

SensorStatus GNSS::getStatus() const
{
    return status;
};

GNSS::HealthDiagnostics GNSS::getHealthDiagnostics() const
{
    HealthDiagnostics diag;
    diag.hasEverFixed = hasEverFixed;
    diag.msSinceLastPvt = millis() - lastPvtMillis;
    diag.consecutiveSuccesses = consecutiveSuccesses;
    diag.consecutiveFailures = consecutiveFailures;
    return diag;
};

GnssData GNSS::getData(GnssData d)
{
    unsigned long now = millis();
    gnss.getPVT();

    d.valid = gnss.getGnssFixOk() && (gnss.getFixType() >= 3);

    d.latitude = gnss.getLatitude() / 1e7;
    d.longitude = gnss.getLongitude() / 1e7;
    d.altitudeMSL = gnss.getAltitudeMSL() / 1000.0f;

    d.velNorth = gnss.getNedNorthVel() / 1000.0f;
    d.velEast = gnss.getNedEastVel() / 1000.0f;
    d.velDown = gnss.getNedDownVel() / 1000.0f;

    d.horizontalAcc = gnss.getHorizontalAccuracy() / 1000.0f;
    d.verticalAcc = gnss.getVerticalAccuracy() / 1000.0f;
    d.speedAcc = gnss.getSpeedAccEst() / 1000.0f;

    d.fixType = gnss.getFixType();
    d.satellites = gnss.getSIV();
    d.pdop = gnss.getPDOP() / 100.0f;

    d.timeOfWeekMs = gnss.getTimeOfWeek();

    bool timeAdvanced = !hasTimeOfWeek || (d.timeOfWeekMs != lastTimeOfWeekMs);
    if (timeAdvanced)
    {
        hasTimeOfWeek = true;
        lastTimeOfWeekMs = d.timeOfWeekMs;
        recordFreshPvt(d.valid, now);
    }
    else
    {
        recordPvtSilence(now);
    }

    return d;
}