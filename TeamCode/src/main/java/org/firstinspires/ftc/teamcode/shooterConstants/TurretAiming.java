package org.firstinspires.ftc.teamcode.shooterConstants;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * TurretAiming
 *
 * Turret control that aims at the goal using:
 * 1. Pinpoint odometry (primary) - knows where robot is, calculates angle to goal
 * 2. Velocity compensation (shooting while moving) - leads the target based on robot velocity
 * 3. Limelight (optional backup) - corrects if odometry drifts
 *
 * HOW SHOOTING-WHILE-MOVING WORKS:
 * ----------------------------------
 * When the robot is moving, the projectile will be imparted with the robot's velocity.
 * If we aim directly at the goal, the shot will miss because by the time the projectile
 * arrives, the robot has moved. We need to "lead" the target.
 *
 * The algorithm:
 *   1. Estimate time of flight from robot distance to goal (TIME_OF_FLIGHT_SECS)
 *   2. Predict where the robot (turret) will be when the projectile arrives
 *   3. Aim at a "virtual target" offset from the real goal to compensate
 *
 * This is a simplified version - no lookup table, just linear velocity compensation.
 * You can tune TIME_OF_FLIGHT_SECS and PHASE_DELAY_SECS for your robot.
 */
public class TurretAiming {

    // ==================== HARDWARE ====================

    private final DcMotorEx turretMotor;
    private final Limelight3A limelight;

    // ==================== GOAL POSITIONS ====================

    /** RED alliance goal (x, y in inches) - TODO: SET THESE! */
    private static final Pose RED_GOAL = new Pose(140.53044654939106, 139.9445196211096);

    /** BLUE alliance goal (x, y in inches) - TODO: SET THESE! */
    private static final Pose BLUE_GOAL = new Pose(13.9695534506, 139.9445196211096);

    private Alliance currentAlliance = Alliance.BLUE;

    public enum Alliance {
        RED, BLUE
    }

    // ==================== TURRET CONSTRAINTS ====================
    // TODO: MEASURE AND SET THESE BASED ON YOUR PHYSICAL LIMITS!

    /** Minimum turret angle (radians) - robot-relative, can't go further counter-clockwise */
    private static final double TURRET_MIN_ANGLE_RAD = Math.toRadians(-96);

    /** Maximum turret angle (radians) - robot-relative, can't go further clockwise */
    private static final double TURRET_MAX_ANGLE_RAD = Math.toRadians(90);

    /** Turret home position (radians, typically 0 = straight forward) */
    private static final double TURRET_HOME_ANGLE_RAD = 0.0;

    // ==================== TURRET MOTOR CALIBRATION ====================
    // TODO: CALIBRATE THIS!

    /** How many encoder ticks per radian of turret rotation */
    private static final double TURRET_TICKS_PER_RADIAN = 427.8880645026 / (90.0 / 53.8);

    // ==================== CONTROL CONSTANTS ====================

    /** Power when moving turret to target */
    private static final double TURRET_MOVE_P = 0.005;

    /** Maximum power for large turret movements (slew) */
    private static final double TURRET_SLEW_POWER_LIMIT = 0.57;

    /** Minimum power to hold position (prevent drift) */
    private static final double TURRET_HOLD_POWER = 0.052;

    /** Proportional gain for position holding */
    private static final double TURRET_HOLD_KP = 0.003;

    /** How close to target counts as "at target" (ticks) */
    private static final int TURRET_TOLERANCE_TICKS = 10;

    /**
     * Tighter tolerance used specifically for zeroing before a vision correction cycle.
     * We want the turret to be genuinely close to zero — within ~3 degrees —
     * before we trust the Limelight pose. Loosen or tighten to taste.
     *
     * At TURRET_TICKS_PER_RADIAN ≈ 255 ticks/rad, 3 degrees ≈ 13 ticks.
     */
    private static final int TURRET_ZERO_TOLERANCE_TICKS = 13;

    /** Limelight proportional gain for fine adjustment */
    private static final double LIMELIGHT_KP = 0.0065;

    /** Minimum power when using Limelight */
    private static final double LIMELIGHT_MIN_POWER = 0.085;

    // ==================== SHOOTING-WHILE-MOVING CONSTANTS ====================

    /**
     * Time-of-flight lookup table: distance (inches) → flight time (seconds).
     *
     * The projectile takes longer to reach the goal at greater distances, so using
     * a flat constant would over-compensate up close and under-compensate far away.
     * This table interpolates between measured data points for accuracy at all ranges.
     *
     * HOW TO FILL THIS IN:
     *   1. Position robot at a known distance from goal (measure with tape)
     *   2. Shoot while stationary — confirm shots are landing in goal
     *   3. Drive at a known, constant speed and shoot
     *   4. Adjust TIME_OF_FLIGHT value at that distance until shots are consistent
     *   5. Repeat at several distances (suggest: 24, 48, 72, 96, 120 inches)
     *
     * The table will linearly interpolate between your data points, and clamp
     * (hold the nearest edge value) for distances outside the table range.
     *
     * Format: { distanceInches, timeOfFlightSeconds }
     *
     * TODO: REPLACE THESE PLACEHOLDER VALUES WITH YOUR MEASURED DATA!
     */
    private static final double[][] TIME_OF_FLIGHT_TABLE = {
            //  dist (in)   TOF (sec)
            {   40.0,       0.3 },
            {   42.0,       0.3 },
            {   44.0,       0.3 },
            {   46.0,       0.3 },
            {   48.0,       0.3 },
            {   70.0,       0.45 },
            {   72.0,       0.45 },
            {   74.0,       0.45 },
            {   76.0,       0.45 },
            {   78.0,       0.45 },
            {   91.0,       0.49 },
            {   93.0,       0.49 },
            {   95.0,       0.49 },
            {   97.0,       0.49 },
            {   99.0,       0.51 },
            {  132.0,       0.75 },
            {  134.0,       0.75 },
            {  136.0,       0.78 },
            {  138.0,       0.78 },
            {  151.0,       0.85 },
    };

    /**
     * Phase delay in seconds - accounts for sensor/control loop latency.
     * The robot's actual position when the shot fires is slightly ahead of
     * what the odometry reported when we started calculating.
     * TODO: TUNE THIS. Typically 0.02–0.05 seconds.
     */
    private static final double PHASE_DELAY_SECS = 0.02;

    /**
     * Number of lookahead iterations to converge on the correct virtual target.
     * More iterations = more accurate but slightly more CPU. 5 is plenty.
     */
    private static final int LOOKAHEAD_ITERATIONS = 5;

    /**
     * Enable or disable shooting-while-moving compensation.
     * Set to false to revert to simple static aiming (useful for debugging).
     */
    private boolean useVelocityCompensation = true;

    /** Cached TOF value from last update — exposed for telemetry */
    private double lastTimeOfFlight = 0.0;

    // ==================== VELOCITY TRACKING ====================

    /** Previous robot pose for velocity estimation */
    private Pose lastRobotPose = null;

    /** Timestamp of last update (nanoseconds) for velocity calculation */
    private long lastUpdateTimeNs = 0;

    /** Estimated robot velocity in field frame (inches/second), X component */
    private double robotVelocityX = 0.0;

    /** Estimated robot velocity in field frame (inches/second), Y component */
    private double robotVelocityY = 0.0;

    /**
     * Low-pass filter coefficient for velocity smoothing (0 = no filtering, 1 = full filtering).
     * Higher values smooth out noise but add lag. 0.8 is a good starting point.
     */
    private static final double VELOCITY_FILTER_ALPHA = 0.9;

    // ==================== STATE ====================

    private boolean isActive = false;
    private boolean useLimelightCorrection = false;
    private boolean isGoingHome = false;

    private double targetAngleFieldRelative = 0.0;
    private double targetAngleRobotRelative = 0.0;
    private int targetTicks = 0;
    private boolean lastTargetWasSafe = true;

    /** The computed lookahead (virtual) target position - exposed for telemetry */
    private double lookaheadTargetX = 0.0;
    private double lookaheadTargetY = 0.0;

    // ==================== CONSTRUCTOR ====================

    /**
     * Create turret aiming controller
     *
     * @param turretMotor The turret rotation motor
     * @param limelight   The Limelight camera (can be null if not using)
     */
    public TurretAiming(DcMotorEx turretMotor, Limelight3A limelight) {
        this.turretMotor = turretMotor;
        this.limelight = limelight;
    }

    // ==================== MAIN UPDATE METHOD ====================

    /**
     * Call this every loop!
     *
     * @param robotPose    Current robot position from odometry
     * @param robotHeading Current robot heading (radians)
     */
    public void update(Pose robotPose, double robotHeading) {
        if (!isActive) {
            turretMotor.setPower(0);
            return;
        }

        // Always update velocity estimate (needed even when going home, in case we re-enable)
        updateVelocityEstimate(robotPose);

        // If going home, bypass odometry calculation and drive straight to tick 0
        if (isGoingHome) {
            targetTicks = (int) (TURRET_HOME_ANGLE_RAD * TURRET_TICKS_PER_RADIAN);
            updateWithOdometry();
            if (isAtTarget()) {
                isGoingHome = false;
            }
            return;
        }

        // --- Step 1: Phase-delay compensation ---
        // Account for latency: estimate where the robot will actually be when the
        // shot fires (a tiny fraction of a second in the future).
        double phasePoseX = robotPose.getX() + robotVelocityX * PHASE_DELAY_SECS;
        double phasePoseY = robotPose.getY() + robotVelocityY * PHASE_DELAY_SECS;

        // --- Step 2: Iterative lookahead to find virtual target ---
        // We need to find the point in space to aim at so that the projectile
        // arrives at the goal despite the robot's motion imparting velocity to it.
        //
        // Algorithm (from LaunchCalculator reference):
        //   a) Compute time of flight at current distance
        //   b) Predict where the turret will be after that time (using robot velocity)
        //   c) Re-compute distance from that predicted position to the goal
        //   d) Repeat until it converges (~5 iterations is enough)

        Pose goal = getCurrentGoal();
        double virtualTargetX = goal.getX();
        double virtualTargetY = goal.getY();

        if (useVelocityCompensation) {
            double lookaheadX = phasePoseX;
            double lookaheadY = phasePoseY;

            for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
                // Compute current distance from lookahead position to goal
                double ldx = goal.getX() - lookaheadX;
                double ldy = goal.getY() - lookaheadY;
                double lookaheadDist = Math.sqrt(ldx * ldx + ldy * ldy);

                // Look up time of flight for THIS distance (re-queried each iteration
                // so it converges correctly, just like LaunchCalculator does)
                lastTimeOfFlight = getTimeOfFlight(lookaheadDist);

                // Predict robot position when projectile arrives
                lookaheadX = phasePoseX + robotVelocityX * lastTimeOfFlight;
                lookaheadY = phasePoseY + robotVelocityY * lastTimeOfFlight;
            }

            // Aim from the converged lookahead position toward the real goal
            phasePoseX = lookaheadX;
            phasePoseY = lookaheadY;
        }

        lookaheadTargetX = virtualTargetX;
        lookaheadTargetY = virtualTargetY;

        // --- Step 3: Field-relative angle from (predicted) robot to goal ---
        double dx = virtualTargetX - phasePoseX;
        double dy = virtualTargetY - phasePoseY;
        double fieldAngleToGoal = Math.atan2(dy, dx);

        // --- Step 4: Store field-relative angle for telemetry ---
        targetAngleFieldRelative = fieldAngleToGoal;

        // --- Step 5: Convert to robot-relative angle ---
        double robotRelativeAngle = normalizeAngle(fieldAngleToGoal - robotHeading);

        // --- Step 6: Check physical limits ---
        lastTargetWasSafe = (robotRelativeAngle >= TURRET_MIN_ANGLE_RAD
                && robotRelativeAngle <= TURRET_MAX_ANGLE_RAD);

        // --- Step 7: Clamp to physical limits ---
        robotRelativeAngle = Math.max(TURRET_MIN_ANGLE_RAD,
                Math.min(TURRET_MAX_ANGLE_RAD, robotRelativeAngle));

        targetAngleRobotRelative = robotRelativeAngle;

        // --- Step 8: Convert to motor ticks ---
        targetTicks = (int) (robotRelativeAngle * TURRET_TICKS_PER_RADIAN);

        // --- Step 9: Drive motor ---
        if (useLimelightCorrection && limelight != null) {
            updateWithLimelight();
        } else {
            updateWithOdometry();
        }
    }

    // ==================== TIME OF FLIGHT LOOKUP ====================

    /**
     * Interpolates time of flight (seconds) from the lookup table for a given
     * distance (inches). Clamps to edge values outside the table range.
     *
     * Uses linear interpolation between the two nearest data points.
     */
    private double getTimeOfFlight(double distanceInches) {
        if (distanceInches <= TIME_OF_FLIGHT_TABLE[0][0]) {
            return TIME_OF_FLIGHT_TABLE[0][1];
        }
        if (distanceInches >= TIME_OF_FLIGHT_TABLE[TIME_OF_FLIGHT_TABLE.length - 1][0]) {
            return TIME_OF_FLIGHT_TABLE[TIME_OF_FLIGHT_TABLE.length - 1][1];
        }
        for (int i = 0; i < TIME_OF_FLIGHT_TABLE.length - 1; i++) {
            double d0 = TIME_OF_FLIGHT_TABLE[i][0];
            double d1 = TIME_OF_FLIGHT_TABLE[i + 1][0];
            if (distanceInches >= d0 && distanceInches <= d1) {
                double t = (distanceInches - d0) / (d1 - d0);
                return TIME_OF_FLIGHT_TABLE[i][1]
                        + t * (TIME_OF_FLIGHT_TABLE[i + 1][1] - TIME_OF_FLIGHT_TABLE[i][1]);
            }
        }
        return TIME_OF_FLIGHT_TABLE[TIME_OF_FLIGHT_TABLE.length - 1][1];
    }

    /** Get the time of flight used in the last update() call — for telemetry */
    public double getLastTimeOfFlight() {
        return lastTimeOfFlight;
    }

    // ==================== VELOCITY ESTIMATION ====================

    private void updateVelocityEstimate(Pose robotPose) {
        long nowNs = System.nanoTime();

        if (lastRobotPose != null && lastUpdateTimeNs != 0) {
            double dtSecs = (nowNs - lastUpdateTimeNs) / 1_000_000_000.0;

            if (dtSecs > 0.001 && dtSecs < 0.5) {
                double rawVx = (robotPose.getX() - lastRobotPose.getX()) / dtSecs;
                double rawVy = (robotPose.getY() - lastRobotPose.getY()) / dtSecs;

                robotVelocityX = VELOCITY_FILTER_ALPHA * robotVelocityX
                        + (1.0 - VELOCITY_FILTER_ALPHA) * rawVx;
                robotVelocityY = VELOCITY_FILTER_ALPHA * robotVelocityY
                        + (1.0 - VELOCITY_FILTER_ALPHA) * rawVy;
            }
        }

        lastRobotPose = robotPose;
        lastUpdateTimeNs = nowNs;
    }

    // ==================== ODOMETRY-BASED CONTROL ====================

    private void updateWithOdometry() {
        int currentTicks = turretMotor.getCurrentPosition();
        int error = targetTicks - currentTicks;

        if (Math.abs(error) <= TURRET_TOLERANCE_TICKS) {
            double holdPower = error * TURRET_HOLD_KP;
            holdPower = Math.max(-TURRET_HOLD_POWER, Math.min(TURRET_HOLD_POWER, holdPower));
            turretMotor.setPower(holdPower);
            return;
        }

        double power = TURRET_MOVE_P * Math.abs(error) + LIMELIGHT_MIN_POWER;
        power = Math.min(power, TURRET_SLEW_POWER_LIMIT);
        turretMotor.setPower(error > 0 ? power : -power);
    }

    // ==================== LIMELIGHT-BASED CORRECTION ====================

    private void updateWithLimelight() {
        LLResult result = limelight.getLatestResult();

        if (!result.isValid()) {
            updateWithOdometry();
            return;
        }

        double tx = result.getTx();

        if (Math.abs(tx) > 3.0) {
            double power = -tx * LIMELIGHT_KP + Math.signum(-tx) * LIMELIGHT_MIN_POWER;
            turretMotor.setPower(power);
        } else {
            turretMotor.setPower(0);
        }
    }

    // ==================== CONTROL METHODS ====================

    public void startAiming() {
        isActive = true;
        isGoingHome = false;
    }

    public void stopAiming() {
        isActive = false;
        isGoingHome = false;
        turretMotor.setPower(0);
    }

    public void toggleAiming() {
        if (isActive) stopAiming();
        else startAiming();
    }

    public void enableLimelightCorrection() {
        useLimelightCorrection = true;
    }

    public void disableLimelightCorrection() {
        useLimelightCorrection = false;
    }

    public void toggleLimelightCorrection() {
        useLimelightCorrection = !useLimelightCorrection;
    }

    public void goHome() {
        isActive = true;
        isGoingHome = true;
        useLimelightCorrection = false;
        targetAngleFieldRelative = TURRET_HOME_ANGLE_RAD;
        targetAngleRobotRelative = TURRET_HOME_ANGLE_RAD;
        targetTicks = (int) (TURRET_HOME_ANGLE_RAD * TURRET_TICKS_PER_RADIAN);
    }

    /**
     * Enable/disable velocity compensation for shooting while moving.
     * Disable to revert to simple static aiming (good for debugging).
     */
    public void setVelocityCompensation(boolean enabled) {
        useVelocityCompensation = enabled;
        if (!enabled) {
            robotVelocityX = 0;
            robotVelocityY = 0;
        }
    }

    public void toggleVelocityCompensation() {
        setVelocityCompensation(!useVelocityCompensation);
    }

    public boolean isUsingVelocityCompensation() {
        return useVelocityCompensation;
    }

    // ==================== ALLIANCE & GOAL ====================

    public void setAlliance(Alliance alliance) {
        currentAlliance = alliance;
    }

    public Pose getCurrentGoal() {
        return currentAlliance == Alliance.RED ? RED_GOAL : BLUE_GOAL;
    }

    // ==================== STATUS QUERIES ====================

    public boolean isAiming() {
        return isActive;
    }

    public boolean isAtTarget() {
        return Math.abs(targetTicks - turretMotor.getCurrentPosition()) <= TURRET_TOLERANCE_TICKS;
    }

    /**
     * Returns true when the turret is close enough to zero (home) to trust the
     * Limelight for an odometry correction. Uses a tighter tolerance than
     * isAtTarget() to ensure the camera is genuinely forward-facing.
     *
     * The zero tick target is 0 (TURRET_HOME_ANGLE_RAD = 0.0 * ticks = 0).
     */
    public boolean isZeroed() {
        return Math.abs(turretMotor.getCurrentPosition()) <= TURRET_ZERO_TOLERANCE_TICKS;
    }

    public boolean isUsingLimelight() {
        return useLimelightCorrection;
    }

    public double getTargetAngle() {
        return targetAngleFieldRelative;
    }

    public double getTargetAngleRobotRelative() {
        return targetAngleRobotRelative;
    }

    public double getCurrentAngle() {
        return getCurrentTicks() / TURRET_TICKS_PER_RADIAN;
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public int getCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    public int getError() {
        return targetTicks - turretMotor.getCurrentPosition();
    }

    public boolean isTargetAngleSafe() {
        return lastTargetWasSafe;
    }

    public double getRobotVelocityX() { return robotVelocityX; }
    public double getRobotVelocityY() { return robotVelocityY; }
    public double getLookaheadTargetX() { return lookaheadTargetX; }
    public double getLookaheadTargetY() { return lookaheadTargetY; }

    // ==================== UTILITY ====================

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ==================== TELEMETRY HELPERS ====================

    public String getStatusString() {
        if (!isActive) return "INACTIVE";
        if (isGoingHome) return "GOING HOME";
        if (isAtTarget()) return "AT TARGET";
        return "MOVING";
    }

    public String getControlMethodString() {
        if (!isActive) return "N/A";
        if (isGoingHome) return "HOME";
        if (useLimelightCorrection) return "LIMELIGHT";
        if (useVelocityCompensation) return "ODOMETRY+VEL";
        return "ODOMETRY";
    }
}