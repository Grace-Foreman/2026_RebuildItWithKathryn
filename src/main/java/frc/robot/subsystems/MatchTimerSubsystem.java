package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
 
import java.util.Map;
import java.util.Optional;
 
/**
 * MatchTimerSubsystem
 *
 * Publishes FRC match phase, hub activation status, and alliance
 * to both SmartDashboard and Shuffleboard every robot periodic cycle.
 *
 * Hub activates at endgame (30 s remaining in teleop).
 * A 10-second warning fires at 40 s remaining.
 *
 * Drop MatchTimerSubsystem.java into src/main/java/frc/robot/subsystems/
 * then register it in RobotContainer (see bottom of this file).
 */
public class MatchTimerSubsystem extends SubsystemBase {
 
    // ---------------------------------------------------------------
    // Timing constants (seconds remaining in match)
    // ---------------------------------------------------------------
    private static final double ENDGAME_THRESHOLD  = 30.0;  // hub goes active
    private static final double HUB_WARNING_BUFFER = 10.0;  // warn this many seconds before
 
    // ---------------------------------------------------------------
    // Shuffleboard entries — written once in constructor
    // ---------------------------------------------------------------
    private final GenericEntry sbMatchTime;
    private final GenericEntry sbPhase;
    private final GenericEntry sbHubActive;
    private final GenericEntry sbHubWarning;
    private final GenericEntry sbHubStatus;
    private final GenericEntry sbAlliance;
    private final GenericEntry sbRoleHint;
 
    // ---------------------------------------------------------------
    // State
    // ---------------------------------------------------------------
    private boolean isOffenseMode = true;   // toggle via SmartDashboard chooser or command
    private String  lastPhase     = "";
 
    public MatchTimerSubsystem() {
        // ── Shuffleboard tab setup ─────────────────────────────────
        ShuffleboardTab tab = Shuffleboard.getTab("Match Timer");
 
        // Left column: phase + timer
        ShuffleboardLayout timeLayout = tab
            .getLayout("Match Clock", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0)
            .withProperties(Map.of("Label position", "TOP"));
 
        sbMatchTime = timeLayout
            .add("Time Remaining", 150.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0, "max", 150, "num tick marks", 6))
            .getEntry();
 
        sbPhase = timeLayout
            .add("Phase", "WAITING")
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
 
        sbAlliance = timeLayout
            .add("Alliance", "Unknown")
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
 
        // Middle column: hub status
        ShuffleboardLayout hubLayout = tab
            .getLayout("Hub Status", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withPosition(2, 0)
            .withProperties(Map.of("Label position", "TOP"));
 
        sbHubActive = hubLayout
            .add("Hub Active", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "gray"))
            .getEntry();
 
        sbHubWarning = hubLayout
            .add("Hub Warning (10s)", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "orange", "colorWhenFalse", "gray"))
            .getEntry();
 
        sbHubStatus = hubLayout
            .add("Hub Message", "Inactive — activates at 0:30")
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
 
        // Right column: role hint
        sbRoleHint = tab
            .add("Strategy Tip", "")
            .withWidget(BuiltInWidgets.kTextView)
            .withSize(3, 1)
            .withPosition(0, 4)
            .getEntry();
 
        // Also expose offense/defense toggle on dashboard so drive team can flip it
        SmartDashboard.putBoolean("Offense Mode", isOffenseMode);
    }
 
    // ---------------------------------------------------------------
    // Called every ~20 ms by the scheduler
    // ---------------------------------------------------------------
    @Override
    public void periodic() {
        // Read current match time (counts down from 150 → 0 during teleop,
        // returns -1 when not in a match — treat as 150 for display)
        double rawTime = DriverStation.getMatchTime();
        double timeLeft = (rawTime < 0) ? 150.0 : rawTime;
 
        // Allow drive team to flip offense/defense on the fly
        isOffenseMode = SmartDashboard.getBoolean("Offense Mode", true);
 
        // ── Phase ─────────────────────────────────────────────────
        String phase = computePhase(timeLeft);
 
        // ── Hub flags ─────────────────────────────────────────────
        boolean inTeleop   = DriverStation.isTeleop() || DriverStation.isTeleopEnabled();
        boolean hubActive  = inTeleop && timeLeft <= ENDGAME_THRESHOLD;
        boolean hubWarning = inTeleop
                          && timeLeft > ENDGAME_THRESHOLD
                          && timeLeft <= (ENDGAME_THRESHOLD + HUB_WARNING_BUFFER);
 
        String hubMsg = buildHubMessage(timeLeft, hubActive, hubWarning, inTeleop);
        String tip    = buildStrategyTip(phase, hubActive, hubWarning, timeLeft);
 
        // ── Alliance ──────────────────────────────────────────────
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        String allianceStr = allianceOpt.map(Alliance::name).orElse("Unknown");
 
        // ── Push to Shuffleboard ──────────────────────────────────
        sbMatchTime .setDouble(timeLeft);
        sbPhase     .setString(phase);
        sbAlliance  .setString(allianceStr);
        sbHubActive .setBoolean(hubActive);
        sbHubWarning.setBoolean(hubWarning);
        sbHubStatus .setString(hubMsg);
        sbRoleHint  .setString(tip);
 
        // ── Push summary to SmartDashboard as backup ──────────────
        SmartDashboard.putNumber("Match Time",    timeLeft);
        SmartDashboard.putString("Match Phase",   phase);
        SmartDashboard.putBoolean("Hub Active",   hubActive);
        SmartDashboard.putBoolean("Hub Warning",  hubWarning);
        SmartDashboard.putString("Hub Status",    hubMsg);
        SmartDashboard.putString("Alliance",      allianceStr);
        SmartDashboard.putString("Strategy Tip",  tip);
 
        lastPhase = phase;
    }
 
    // ---------------------------------------------------------------
    // Public helpers (usable from commands)
    // ---------------------------------------------------------------
 
    /** True when the hub has just become active this cycle. */
    public boolean isHubActive() {
        double t = DriverStation.getMatchTime();
        return DriverStation.isTeleopEnabled() && t >= 0 && t <= ENDGAME_THRESHOLD;
    }
 
    /** True during the 10-second window before hub activation. */
    public boolean isHubWarningActive() {
        double t = DriverStation.getMatchTime();
        return DriverStation.isTeleopEnabled()
            && t > ENDGAME_THRESHOLD
            && t <= (ENDGAME_THRESHOLD + HUB_WARNING_BUFFER);
    }
 
    /** Seconds remaining in match, or 150 if not in match. */
    public double getMatchTimeRemaining() {
        double t = DriverStation.getMatchTime();
        return t < 0 ? 150.0 : t;
    }
 
    /** Set scoring/defense mode from a command or button binding. */
    public void setOffenseMode(boolean offense) {
        isOffenseMode = offense;
        SmartDashboard.putBoolean("Offense Mode", offense);
    }
 
    // ---------------------------------------------------------------
    // Private helpers
    // ---------------------------------------------------------------
 
    private String computePhase(double timeLeft) {
        if (DriverStation.isAutonomous() || DriverStation.isAutonomousEnabled()) {
            return "AUTO";
        } else if (DriverStation.isTeleop() || DriverStation.isTeleopEnabled()) {
            return timeLeft <= ENDGAME_THRESHOLD ? "ENDGAME" : "TELEOP";
        } else if (DriverStation.isDisabled()) {
            return "DISABLED";
        }
        return "WAITING";
    }
 
    private String buildHubMessage(double timeLeft, boolean active, boolean warning, boolean inTeleop) {
        if (active) {
            return isOffenseMode ? "HUB ACTIVE — SCORE NOW!" : "HUB ACTIVE — clear hub zone!";
        } else if (warning) {
            int secsUntil = (int) Math.ceil(timeLeft - ENDGAME_THRESHOLD);
            return isOffenseMode
                ? "Hub in " + secsUntil + "s — get into position!"
                : "Hub in " + secsUntil + "s — exit hub zone!";
        } else if (inTeleop) {
            return "Inactive — activates at 0:30";
        }
        return "Inactive";
    }
 
    private String buildStrategyTip(String phase, boolean hubActive, boolean hubWarning, double timeLeft) {
        if (phase.equals("AUTO")) {
            return isOffenseMode ? "Auto: score preloaded game pieces" : "Auto: position for defense";
        }
        if (hubActive) {
            return isOffenseMode
                ? "ENDGAME: hub is open — cycle fast!"
                : "ENDGAME: defend feeding station, respect hub zone";
        }
        if (hubWarning) {
            return isOffenseMode
                ? "Hub soon — approach now, avoid fouls"
                : "Hub soon — start retreating from hub";
        }
        if (timeLeft > 60) {
            return isOffenseMode ? "Teleop: build cycle efficiency" : "Defense: 5s pin limit, stay legal";
        }
        return isOffenseMode ? "Final minute: maximize cycle count" : "Final minute: protect your scorers";
    }
}
 