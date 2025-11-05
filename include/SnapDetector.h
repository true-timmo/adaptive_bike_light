#ifndef _SnapDetector_h_
#define _SnapDetector_h_

#include <Arduino.h>

class SnapDetector {
    private:
        static constexpr float    YAW_SNAP_THR_DEGS     = 11.0f;  // Peak je Vorzeichenlobe
        static constexpr float    YAW_SNAP_ENERGY_DEG   = 8.0f;   // ∫|yaw| dt in Grad
        static constexpr float    YAW_SNAP_JERK_THR     = 3500.0f;// °/s² (deutlich höher)
        static constexpr uint32_t SNAP_MIN_PHASE_MS     = 70;     // mind. Dauer des vorherigen Lobes
        static constexpr uint32_t SNAP_REFRACTORY_MS    = 300;    // Sperrzeit nach Snap
        static constexpr uint32_t SNAP_HOLD_MS          = 150;    // Boost-Dauer
        static constexpr float    YAW_EPS               = 2.5f;   // Deadzone um 0°/s

        float    prevYawRate   = 0.0f;
        int      lastNonZeroSign = 0;
        float    peakYawMag    = 0.0f;
        float    yawEnergyDeg  = 0.0f;
        uint32_t phaseStartMs  = 0;
        uint32_t lastSnapAt    = 0;
        uint32_t snapHoldUntil = 0;

        uint32_t& now;
        float&    dtRef;
        Stream*   logger;

    public:
        SnapDetector(Stream* l, uint32_t& currentTimestamp, float& lastToCurrent)
        : now(currentTimestamp), dtRef(lastToCurrent), logger(l) {}

        bool snapDetected(float yawRate) {
            // 0) Refractory
            if (now - lastSnapAt < SNAP_REFRACTORY_MS) {
                prevYawRate = yawRate;
                return (now < snapHoldUntil);
            }

            // 1) Signum mit Deadzone
            int sign = 0;
            if (yawRate >  YAW_EPS) sign = +1;
            if (yawRate < -YAW_EPS) sign = -1;

            // 2) dt stabilisieren
            float dt = dtRef;
            if (dt <= 0.0f)  dt = 0.01f;
            if (dt < 0.002f) dt = 0.002f;

            // 3) Wenn wir innerhalb eines laufenden (nicht-null) Lobes sind: Peak & Energie aufsummieren
            if (sign != 0 && sign == lastNonZeroSign) {
                float mag = fabsf(yawRate);
                if (mag > peakYawMag) peakYawMag = mag;
                yawEnergyDeg += mag * dt;   // °/s * s = °
            }

            // 4) Richtungswechsel? (nur wenn beide Seiten !=0 und unterschiedlich)
            if (sign != 0 && lastNonZeroSign != 0 && sign != lastNonZeroSign) {
                uint32_t phaseDur = now - phaseStartMs;
                float jerk = fabsf((yawRate - prevYawRate) / dt);

                bool enoughDuration = (phaseDur >= SNAP_MIN_PHASE_MS);
                bool enoughEnergy   = (yawEnergyDeg >= YAW_SNAP_ENERGY_DEG);
                bool strongPeak     = (peakYawMag >= YAW_SNAP_THR_DEGS);
                bool strongJerk     = (jerk       >= YAW_SNAP_JERK_THR);

                if (enoughDuration && enoughEnergy && (strongPeak || strongJerk)) {
                snapHoldUntil = now + SNAP_HOLD_MS;
                lastSnapAt    = now;
                if (logger) logger->printf("SNAP peak=%.1f E=%.1f dur=%ums jerk=%.0f\n",
                                            peakYawMag, yawEnergyDeg, phaseDur, jerk);
                }

                // neue Phase starten (mit aktuellem Signum, NICHT 0)
                lastNonZeroSign = sign;
                peakYawMag      = fabsf(yawRate);
                yawEnergyDeg    = peakYawMag * dt;
                phaseStartMs    = now;
            }

            // 5) Phasenstart, wenn wir erstmals ein nicht-null Signum betreten
            if (sign != 0 && lastNonZeroSign == 0) {
                lastNonZeroSign = sign;
                phaseStartMs    = now;
                float mag = fabsf(yawRate);
                peakYawMag      = mag;
                yawEnergyDeg    = mag * dt;
            }
            prevYawRate = yawRate;

            return (now < snapHoldUntil);
        }
};
#endif // _SnapDetector_h_