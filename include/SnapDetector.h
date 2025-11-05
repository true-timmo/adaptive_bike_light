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

        float prevYawRate      = 0.0f;
        int      prevYawSign   = 0;
        float    peakYawMag    = 0.0f;     // max(|yaw|) im aktuellen Lobe
        float    yawEnergyDeg  = 0.0f;     // ∫|yaw| dt im aktuellen Lobe
        uint32_t phaseStartMs  = 0;        // Startzeit des aktuellen Lobes
        uint32_t lastSnapAt    = 0;        // für Refractory
        uint32_t snapHoldUntil = 0;
        
        uint32_t& now;
        float& dtRef;
        Stream* logger;

    public:
        SnapDetector(Stream* l, uint32_t& currentTimestamp, float& lastToCurrent)
            : logger(l), now(currentTimestamp), dtRef(lastToCurrent) {}

        bool snapDetected(float& yawRate) {
            // Refractory: kurz nach Snap nicht erneut triggern
            if (now - lastSnapAt < SNAP_REFRACTORY_MS) {
                prevYawRate = yawRate;
                return (now < snapHoldUntil);
            }

            // Signum mit Deadzone
            int sign = 0;
            if (yawRate >  YAW_EPS) sign = +1;
            if (yawRate < -YAW_EPS) sign = -1;

            // dt stabilisieren (Jerk-Noise vermeiden)
            float dt = dtRef;
            if (dt <= 0.0f) dt = 0.01f;
            if (dt < 0.002f) dt = 0.002f; // Schutz: unrealistisch kleine dt clampen

            // Innerhalb der aktuellen Vorzeichenphase: Peak & Energie akkumulieren
            if (sign != 0 && sign == prevYawSign) {
                float mag = fabsf(yawRate);
                if (mag > peakYawMag) peakYawMag = mag;
                yawEnergyDeg += mag * dt;  // °/s * s = °
            }

            // Richtungswechsel?
            if (sign != 0 && prevYawSign != 0 && (sign != prevYawSign)) {
                uint32_t phaseDur = now - phaseStartMs;
                float jerk = fabsf((yawRate - prevYawRate) / dt);

                bool enoughDuration = (phaseDur >= SNAP_MIN_PHASE_MS);
                bool enoughEnergy   = (yawEnergyDeg >= YAW_SNAP_ENERGY_DEG);
                bool strongPeak     = (peakYawMag >= YAW_SNAP_THR_DEGS);
                bool strongJerk     = (jerk       >= YAW_SNAP_JERK_THR);

                if (enoughDuration && enoughEnergy && (strongPeak || strongJerk)) {
                    snapHoldUntil = now + SNAP_HOLD_MS;
                    lastSnapAt    = now;
                    logger->printf("SNAP peak=%.1f E=%.1f dur=%ums jerk=%.0f\n",
                                   peakYawMag, yawEnergyDeg, phaseDur, jerk);
                }

                // neue Phase initialisieren
                peakYawMag    = fabsf(yawRate);
                yawEnergyDeg  = peakYawMag * dt;
                phaseStartMs  = now;
            }

            // Phasenwechsel initialisieren (Start einer Lobe)
            if (sign != prevYawSign) {
                prevYawSign = sign;
                phaseStartMs = now;
                if (sign == 0) { peakYawMag = 0.0f; yawEnergyDeg = 0.0f; }
                else {
                    float mag = fabsf(yawRate);
                    peakYawMag   = mag;
                    yawEnergyDeg = mag * dt;
                }
            }

            prevYawRate = yawRate;
            return (now < snapHoldUntil);
        };
};
#endif // _SnapDetector_h_