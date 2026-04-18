// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shoot subsystem for Team 6875 — 2026 season.
 *
 * Hardware:
 *   Flywheel — Kraken X60 TalonFX, CAN ID 16
 *   Feeder   — Kraken X60 TalonFX, CAN ID 15
 *
 * ── Speed calculation: piecewise linear interpolation ────────────────────────
 * The field is divided into 4 distance zones. Each zone has a tuned RPS value.
 * When the robot is between two zones, RPS is smoothly interpolated.
 *
 *   Zone 1: 0   – 1.5 m  →  kZone1RPS
 *   Zone 2: 1.5 – 3.0 m  →  kZone2RPS
 *   Zone 3: 3.0 – 4.5 m  →  kZone3RPS
 *   Zone 4: 4.5 – 6.0 m  →  kZone4RPS
 *
 * Example: robot is at 2.25 m (halfway between zone 2 and zone 3 breakpoints)
 *   → RPS = lerp(kZone2RPS, kZone3RPS, 0.5)
 *
 * ── How to tune ──────────────────────────────────────────────────────────────
 *  1. Stand at each zone boundary distance (1.5m, 3.0m, 4.5m, 6.0m).
 *  2. Set all zones to the same RPS and shoot.
 *  3. Adjust each zone's RPS constant until shots land at that distance.
 *  4. Intermediate distances will automatically interpolate — no extra tuning.
 */
public class SmartShoot extends SubsystemBase {

    public static class ShootConstants {

        // ── CAN IDs ───────────────────────────────────────────────────────────
        public static final int kFlywheelCanId = 16;
        public static final int kFeederCanId   = 15;

        // ── Flywheel PID (Slot 0) ─────────────────────────────────────────────
        // Tune kV first: kV ≈ 1 / free_speed_RPS. Kraken X60 ≈ 100 RPS → 0.01
        // If flywheel tracks target well, leave kP as-is.
        // If it oscillates, lower kP. If it's slow to reach target, raise kP.
        public static final double kP = 0.10;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kS = 0.10;  // Static friction
        public static final double kV = 0.12;  // Velocity feedforward — tune this first

        // ── Zone breakpoint distances (meters) ────────────────────────────────
        // These define the boundaries between zones.
        // Adjust to match where you actually test shots on your field.
        public static final double kZone1Distance = 2; // meters — end of zone 1
        public static final double kZone2Distance = 3.5; // meters — end of zone 2
        public static final double kZone3Distance = 5; // meters — end of zone 3
        public static final double kZone4Distance = 7.5; // meters — end of zone 4 (max range)

        // ── Zone RPS values — TUNE THESE ON THE ROBOT ─────────────────────────
        // Stand at the zone boundary, shoot, adjust until on target.
        // Zone 1 is closest → needs most power (steeper arc).
        // Zone %4 is farthest → needs most power too (more distance).
        // Start all at 30 and adjust from there.
        public static final double kZone1RPS = 40; // At 1.5 m — TUNABLE
        public static final double kZone2RPS = 57; // At 3.0 m — TUNABLE
        public static final double kZone3RPS = 70; // At 4.5 m — TUNABLE
        public static final double kZone4RPS = 84; // At 6.0 m — TUNABLE

        // ── Safety clamp ──────────────────────────────────────────────────────
        public static final double kMaxRPS = 80.0;
        public static final double kMinRPS =  5.0;

        // ── Out-of-range fallback ─────────────────────────────────────────────
        // If robot is closer than zone 1 or farther than zone 4
        public static final double kTooCloseRPS = kZone1RPS; // Just use zone 1 speed
        public static final double kTooFarRPS   = kZone4RPS; // Just use zone 4 speed
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Hardware
    // ─────────────────────────────────────────────────────────────────────────
    private final TalonFX flywheelMotor;
    private final TalonFX feederMotor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut       neutralRequest  = new NeutralOut();

    public SmartShoot() {
        flywheelMotor = new TalonFX(ShootConstants.kFlywheelCanId);
        feederMotor   = new TalonFX(ShootConstants.kFeederCanId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ShootConstants.kP;
        config.Slot0.kI = ShootConstants.kI;
        config.Slot0.kD = ShootConstants.kD;
        config.Slot0.kS = ShootConstants.kS;
        config.Slot0.kV = ShootConstants.kV;

        flywheelMotor.getConfigurator().apply(config);
        flywheelMotor.setNeutralMode(NeutralModeValue.Coast); // Coast down naturally on release

        feederMotor.getConfigurator().apply(config);
        feederMotor.setNeutralMode(NeutralModeValue.Brake);   // Stop immediately on release
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Flywheel
    // ─────────────────────────────────────────────────────────────────────────

    /** Spin flywheel at target RPS using closed-loop PID. */
    public void setFlywheelRPS(double rps) {
        flywheelMotor.setControl(velocityRequest.withVelocity(rps));
    }

    /** Coast flywheel to a stop. */
    public void stopFlywheel() {
        flywheelMotor.setControl(neutralRequest);
    }

    /** @return Current flywheel speed in RPS. */
    public double getFlywheelRPS() {
        return flywheelMotor.getVelocity().getValueAsDouble();
    }

    /**
     * @param targetRPS    Target speed
     * @param toleranceRPS Acceptable error
     * @return true if flywheel is within tolerance of target
     */
    public boolean isUpToSpeed(double targetRPS, double toleranceRPS) {
        return Math.abs(getFlywheelRPS() - targetRPS) <= toleranceRPS;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Feeder
    // ─────────────────────────────────────────────────────────────────────────

    /** Run feeder at given RPS. */
    public void setFeederRPS(double rps) {
        feederMotor.setControl(velocityRequest.withVelocity(rps));
    }

    /** Brake feeder to an immediate stop. */
    public void stopFeeder() {
        feederMotor.setControl(neutralRequest);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Piecewise linear interpolation
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Returns the target flywheel RPS for a given distance using piecewise
     * linear interpolation across the 4 tuned zones.
     *
     * Zone layout:
     *   [0 ──── kZone1Distance] → kZone1RPS
     *   [kZone1Distance ─── kZone2Distance] → lerp(kZone1RPS, kZone2RPS)
     *   [kZone2Distance ─── kZone3Distance] → lerp(kZone2RPS, kZone3RPS)
     *   [kZone3Distance ─── kZone4Distance] → lerp(kZone3RPS, kZone4RPS)
     *   [kZone4Distance ────────────── ∞  ] → kZone4RPS
     *
     * @param distanceMeters Distance to goal in meters
     * @return Target RPS, clamped to [kMinRPS, kMaxRPS]
     */
    public double calculateRPS(double distanceMeters) {
        double rps;

        if (distanceMeters <= ShootConstants.kZone1Distance) {
            // Closer than zone 1 — use zone 1 speed
            rps = ShootConstants.kZone1RPS;

        } else if (distanceMeters <= ShootConstants.kZone2Distance) {
            // Between zone 1 and zone 2
            rps = lerp(ShootConstants.kZone1RPS,     ShootConstants.kZone2RPS,
                       ShootConstants.kZone1Distance, ShootConstants.kZone2Distance,
                       distanceMeters);

        } else if (distanceMeters <= ShootConstants.kZone3Distance) {
            // Between zone 2 and zone 3
            rps = lerp(ShootConstants.kZone2RPS,     ShootConstants.kZone3RPS,
                       ShootConstants.kZone2Distance, ShootConstants.kZone3Distance,
                       distanceMeters);

        } else if (distanceMeters <= ShootConstants.kZone4Distance) {
            // Between zone 3 and zone 4
            rps = lerp(ShootConstants.kZone3RPS,     ShootConstants.kZone4RPS,
                       ShootConstants.kZone3Distance, ShootConstants.kZone4Distance,
                       distanceMeters);

        } else {
            // Farther than zone 4 — use zone 4 speed
            rps = ShootConstants.kZone4RPS;
        }

        return Math.max(ShootConstants.kMinRPS, Math.min(ShootConstants.kMaxRPS, rps));
    }

    /**
     * Linear interpolation helper.
     * Returns the value at `x` between (x0, y0) and (x1, y1).
     */
    private double lerp(double y0, double y1, double x0, double x1, double x) {
        double t = (x - x0) / (x1 - x0); // 0.0 at x0, 1.0 at x1
        return y0 + t * (y1 - y0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Subsystem override
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/RPS", getFlywheelRPS());
    }
}