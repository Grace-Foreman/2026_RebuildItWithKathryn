// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter subsystem for Team 6875 — 2026 season.
 *
 * Hardware:
 *   Flywheel motor (top)  — Kraken X60 TalonFX — launches fuel
 *   Feeder motor  (bottom)— Kraken X60 TalonFX — feeds fuel into flywheel
 *
 * Control scheme:
 *   Operator LB → AlignToGoalCommand (rotate robot + spin up flywheel)
 *   Operator RB → ShootCommand        (spin up flywheel + engage feeder once up to speed)
 */
public class Shoot extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // Constants — tune these to match your robot
    // ─────────────────────────────────────────────────────────────────────────
    public static class ShootConstants {

        // ── TODO: Set CAN IDs to match your robot wiring ─────────────────────
        public static final int kFlywheelMotorCanId = 0; // TODO: Set flywheel (top) motor CAN ID
        public static final int kFeederMotorCanId   = 1; // TODO: Set feeder (bottom) motor CAN ID
        // ─────────────────────────────────────────────────────────────────────

        // Flywheel velocity PID (Slot 0) — tune on robot
        public static final double kFlywheelP = 0.10;  // Proportional gain
        public static final double kFlywheelI = 0.00;  // Integral gain
        public static final double kFlywheelD = 0.00;  // Derivative gain
        public static final double kFlywheelS = 0.10;  // Static feedforward (V)
        public static final double kFlywheelV = 0.12;  // Velocity feedforward (V·s/rot)

        // ── Physics constants for distance-based speed calculation ────────────
        // 2026 game geometry — verify against the game manual
        public static final double kLaunchAngleDeg      = 60.0;   // Fixed launch angle (degrees) — TUNABLE
        public static final double kGoalHeightMeters    = 1.8288; // Hub opening height (72" from floor)
        public static final double kShooterHeightMeters = 0.4826; // Shooter exit height (19" from floor)
        public static final double kWheelRadiusMeters   = 0.0508; // 4" diameter wheel → 2" radius
        public static final double kGearRatio           = 15.34;  // Motor rotations per wheel rotation
        public static final double kEfficiency          = 0.85;   // Launch efficiency (0.0–1.0) — TUNABLE

        // Motor RPS limits — clamps the physics output to safe range
        public static final double kMaxMotorRPS = 95.0;
        public static final double kMinMotorRPS = 10.0;

        // Horizontal range limits — outside these, shooter warns or refuses
        public static final double kMinRangeMeters = 1.0;
        public static final double kMaxRangeMeters = 6.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Hardware
    // ─────────────────────────────────────────────────────────────────────────
    private final TalonFX flywheelMotor; // Top  — spins up to shoot
    private final TalonFX feederMotor;   // Bottom — feeds fuel to flywheel

    // Pre-allocated control requests (avoids GC pressure in periodic loops)
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut       voltageRequest  = new VoltageOut(0);
    private final NeutralOut       neutralRequest  = new NeutralOut();

    // ─────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────
    public Shoot() {
        flywheelMotor = new TalonFX(ShootConstants.kFlywheelMotorCanId);
        feederMotor   = new TalonFX(ShootConstants.kFeederMotorCanId);

        // ── Flywheel: PID velocity control, coasts on neutral ────────────────
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = ShootConstants.kFlywheelP;
        flywheelConfig.Slot0.kI = ShootConstants.kFlywheelI;
        flywheelConfig.Slot0.kD = ShootConstants.kFlywheelD;
        flywheelConfig.Slot0.kS = ShootConstants.kFlywheelS;
        flywheelConfig.Slot0.kV = ShootConstants.kFlywheelV;
        flywheelMotor.getConfigurator().apply(flywheelConfig);
        // Coast: flywheel naturally decelerates when command ends — saves wear
        flywheelMotor.setNeutralMode(NeutralModeValue.Coast);

        // ── Feeder: simple voltage control, brakes on neutral ────────────────
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederMotor.getConfigurator().apply(feederConfig);
        // Brake: feeder stops immediately when command releases it
        feederMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Flywheel (top motor) methods
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Spin flywheel at a precise RPS target using closed-loop PID.
     * Use this when you have a calculated target speed.
     */
    public void setFlywheelRPS(double rps) {
        flywheelMotor.setControl(velocityRequest.withVelocity(rps));
    }

    /**
     * Drive flywheel with a raw voltage.
     * Use as a fallback if PID is not yet tuned.
     */
    public void setVoltageTopShoot(double volts) {
        flywheelMotor.setControl(voltageRequest.withOutput(volts));
    }

    /**
     * Release flywheel to coast down naturally.
     * Preferred over stopTopShoot() because it reduces motor stress.
     * Called when ShootCommand ends.
     */
    public void coastTopShoot() {
        flywheelMotor.setControl(neutralRequest); // NeutralMode.Coast → spins down on its own
    }

    /**
     * Hard-stop the flywheel.
     * Prefer coastTopShoot() in most cases.
     */
    public void stopTopShoot() {
        flywheelMotor.setControl(neutralRequest);
    }

    /** @return Current flywheel velocity in rotations per second. */
    public double getFlywheelRPS() {
        return flywheelMotor.getVelocity().getValueAsDouble();
    }

    /**
     * @param targetRPS    Target speed in rotations per second
     * @param toleranceRPS Acceptable speed error (rotations per second)
     * @return true if flywheel is within tolerance of target speed
     */
    public boolean isUpToSpeed(double targetRPS, double toleranceRPS) {
        return Math.abs(getFlywheelRPS() - targetRPS) <= toleranceRPS;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Feeder (bottom motor) methods
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Drive feeder with a raw voltage to feed fuel into the flywheel.
     * Called by ShootCommand once the flywheel is up to speed.
     */
    public void setVoltageBottomShoot(double volts) {
        feederMotor.setControl(voltageRequest.withOutput(volts));
    }

    /**
     * Hard-stop feeder immediately.
     * Called first when ShootCommand ends (before flywheel coasts).
     */
    public void stopBottomShoot() {
        feederMotor.setControl(neutralRequest); // NeutralMode.Brake → stops quickly
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Physics-based speed calculation
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Calculates the required flywheel motor RPS to hit the goal from a given
     * horizontal distance using projectile motion physics.
     *
     * Formula:
     *   v₀ = sqrt( (g·d²) / (2·cos²θ·(d·tanθ − Δh)) )
     *   wheelRPS = v₀ / (efficiency · 2π · wheelRadius)
     *   motorRPS = wheelRPS · gearRatio
     *
     * @param distanceMeters Horizontal distance to the goal (meters)
     * @return Required motor RPS, clamped to [kMinMotorRPS, kMaxMotorRPS]
     */
    public double calculateMotorRPS(double distanceMeters) {
        double g        = 9.81;
        double d        = distanceMeters;
        double theta    = Math.toRadians(ShootConstants.kLaunchAngleDeg);
        double deltaH   = ShootConstants.kGoalHeightMeters - ShootConstants.kShooterHeightMeters;
        double cosTheta = Math.cos(theta);

        double denominator = 2.0 * cosTheta * cosTheta * (d * Math.tan(theta) - deltaH);
        if (denominator <= 0) return ShootConstants.kMinMotorRPS; // Safety: invalid geometry

        double v0       = Math.sqrt((g * d * d) / denominator);
        double wheelRPS = v0 / (ShootConstants.kEfficiency * 2.0 * Math.PI * ShootConstants.kWheelRadiusMeters);
        double motorRPS = wheelRPS * ShootConstants.kGearRatio;

        return Math.max(ShootConstants.kMinMotorRPS, Math.min(ShootConstants.kMaxMotorRPS, motorRPS));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Subsystem overrides
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Nothing needed here — all updates are driven by commands
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Flywheel RPS",
            this::getFlywheelRPS, null);
        builder.addDoubleProperty("Flywheel Supply Current (A)",
            () -> flywheelMotor.getSupplyCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Feeder Supply Current (A)",
            () -> feederMotor.getSupplyCurrent().getValueAsDouble(), null);
    }
}
