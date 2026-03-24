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
 *   Flywheel motor — Kraken X60 TalonFX, CAN ID 16 — launches fuel
 *   Feeder motor   — Kraken X60 TalonFX, CAN ID 15 — feeds fuel into flywheel
 *
 * Speed units: RPS (TalonFX native unit)
 * Physics calc outputs RPS → plugs directly into VelocityVoltage request.
 */
public class SMARTShoot extends SubsystemBase {

    public static class ShootConstants {

        // CAN IDs
        public static final int kFlywheelCanId = 16; // Flywheel — launches fuel
        public static final int kFeederCanId   = 15; // Feeder   — feeds fuel in

        // Flywheel velocity PID (Slot 0)
        // Tune kV first — it does most of the work for velocity control
        // kV ≈ 1 / free_speed_RPS. Kraken X60 free speed ≈ 100 RPS → kV ≈ 0.01
        public static final double kP = 0.10;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kS = 0.10;  // Static friction feedforward
        public static final double kV = 0.12;  // Velocity feedforward — tune this first

        // RPS limits
        public static final double kMaxRPS = 60.0;
        public static final double kMinRPS = 10.0;

        // ── Physics constants ─────────────────────────────────────────────────
        public static final double kLaunchAngleDeg      = 40.0;   // TUNABLE
        public static final double kGoalHeightMeters    = 1.8288; // 72" hub opening
        public static final double kShooterHeightMeters = 0.4826; // 19" shooter exit
        public static final double kWheelRadiusMeters   = 0.0508; // 4" wheel → 2" radius
        public static final double kGearRatio           = 1.0;    // Direct drive
        public static final double kEfficiency          = 0.7;   // TUNABLE
                                                                   // Overshoot → lower (0.75)
                                                                   // Undershoot → raise (0.95)
        public static final double kMinRangeMeters = 1.0;
        public static final double kMaxRangeMeters = 6.0;
    }

    private final TalonFX flywheelMotor;
    private final TalonFX feederMotor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut       neutralRequest  = new NeutralOut();

    public SMARTShoot() {
        flywheelMotor = new TalonFX(ShootConstants.kFlywheelCanId);
        feederMotor   = new TalonFX(ShootConstants.kFeederCanId);

        // Flywheel: velocity PID, coast on neutral
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = ShootConstants.kP;
        flywheelConfig.Slot0.kI = ShootConstants.kI;
        flywheelConfig.Slot0.kD = ShootConstants.kD;
        flywheelConfig.Slot0.kS = ShootConstants.kS;
        flywheelConfig.Slot0.kV = ShootConstants.kV;
        flywheelMotor.getConfigurator().apply(flywheelConfig);
        flywheelMotor.setNeutralMode(NeutralModeValue.Coast);

        // Feeder: velocity PID (same gains), brake on neutral
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.Slot0.kP = ShootConstants.kP;
        feederConfig.Slot0.kI = ShootConstants.kI;
        feederConfig.Slot0.kD = ShootConstants.kD;
        feederConfig.Slot0.kS = ShootConstants.kS;
        feederConfig.Slot0.kV = ShootConstants.kV;
        feederMotor.getConfigurator().apply(feederConfig);
        feederMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    // ── Flywheel ──────────────────────────────────────────────────────────────

    public void setFlywheelRPS(double rps) {
        flywheelMotor.setControl(velocityRequest.withVelocity(rps));
    }

    public void stopFlywheel() {
        flywheelMotor.setControl(neutralRequest); // Coast
    }

    public double getFlywheelRPS() {
        return flywheelMotor.getVelocity().getValueAsDouble();
    }

    public boolean isUpToSpeed(double targetRPS, double toleranceRPS) {
        return Math.abs(getFlywheelRPS() - targetRPS) <= toleranceRPS;
    }

    // ── Feeder ────────────────────────────────────────────────────────────────

    public void setFeederRPS(double rps) {
        feederMotor.setControl(velocityRequest.withVelocity(rps));
    }

    public void stopFeeder() {
        feederMotor.setControl(neutralRequest); // Brake
    }

    // ── Physics ───────────────────────────────────────────────────────────────

    /**
     * Calculates required flywheel RPS from horizontal distance using projectile physics.
     * v₀ = sqrt((g·d²) / (2·cos²θ·(d·tanθ − Δh)))
     * motorRPS = (v₀ / (efficiency · 2π · wheelRadius)) · gearRatio
     */
    public double calculateRPS(double distanceMeters) {
        double g        = 9.81;
        double theta    = Math.toRadians(ShootConstants.kLaunchAngleDeg);
        double deltaH   = ShootConstants.kGoalHeightMeters - ShootConstants.kShooterHeightMeters;
        double cosTheta = Math.cos(theta);
        double denom    = 2.0 * cosTheta * cosTheta * (distanceMeters * Math.tan(theta) - deltaH);

        if (denom <= 0) return ShootConstants.kMinRPS;

        double v0  = Math.sqrt((g * distanceMeters * distanceMeters) / denom);
        double rps = (v0 / (ShootConstants.kEfficiency * 2.0 * Math.PI * ShootConstants.kWheelRadiusMeters))
                     * ShootConstants.kGearRatio;

        return Math.max(ShootConstants.kMinRPS, Math.min(ShootConstants.kMaxRPS, rps));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/RPS", getFlywheelRPS());
    }
}

