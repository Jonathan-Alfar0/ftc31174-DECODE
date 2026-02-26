package org.firstinspires.ftc.teamcode.AlreadyTourneyReady;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "TOURNAMENT - Blue 12 CLOSE (1)", group = "Autonomous")
@Configurable
public class BallBlueTest12 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private ElapsedTime StateTimer = new ElapsedTime();
    private ElapsedTime advanceTimer = new ElapsedTime();
    private boolean isWaiting = false;
    private int pathState;
    private Paths paths;
    private int lastState = -1;

    /* ================= MECHANISM HARDWARE ================= */
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx turretMotor;
    private Servo turretServo;

    /* ================= SHOOTER CONSTANTS (from TeleOp) ================= */
    private static final double SHOOTER_kP = 225.0;
    private static final double SHOOTER_kI = 11.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 14.0;

    /* ================= SHOT CONFIGURATION ARRAYS ================= */
    // Define RPM for each shot (index 0 = preload, 1 = first pickup shot, etc.)
    private static final double[] SHOT_RPM = {
            1260.0,  // Preload shot
            1260.0,  // Shot 1
            1260.0,  // Shot 2
            1230.0,  // Shot 3
            1475.0,  // Shot 4
            1500.0   // Shot 5
    };

    // Define turret target ticks for each shot
    private static final int[] TURRET_TICKS = {
            -230,   // Preload position
            -15,   // Position 1
            -5,   // Position 2
            230,   // Position 3
            400,   // Position 4
            500    // Position 5
    };

    /* ================= INTAKE CONSTANTS ================= */
    private static final double INTAKE_VELOCITY = 1800.0;

    /* ================= TURRET CONSTANTS ================= */
    private static final double TURRET_HOME = 0.45;
    private static final double TURRET_ACTIVE = 0.2;
    private static final double TURRET_POWER = 0.6;
    private static final double TURRET_HOLD_POWER_MAX = 0.15; // Maximum power to hold position
    private static final double TURRET_HOLD_kP = 0.02; // Proportional gain for holding
    private static final int TURRET_POSITION_TOLERANCE = 10; // Ticks tolerance for "at target"

    /* ================= TURRET STOPPER TIMING ================= */
    private static final double STOPPER_OPEN_TIME = 1.5; // seconds

    /* ================= STATE FLAGS ================= */
    private boolean turretMotorActive = false;
    private boolean turretReachedTarget = false;
    private int currentShotIndex = 0; // Track which shot we're on

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33, 134.5, Math.toRadians(270)));

        // Initialize mechanism hardware
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretServo = hardwareMap.get(Servo.class, "turretStopper");

        // Configure shooter motor with PIDF from TeleOp
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(
                SHOOTER_kP,
                SHOOTER_kI,
                SHOOTER_kD,
                SHOOTER_kF
        );

        // Configure intake motor
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure turret motor
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set turret servo to home position
        turretServo.setDirection(Servo.Direction.REVERSE);
        turretServo.setPosition(TURRET_HOME);

        // Build paths
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized - Shooter PIDF Active");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        // Update turret motor control
        updateTurretMotor();

        // Log telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shot Index", currentShotIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooter Target RPM", getCurrentShooterRPM());
        panelsTelemetry.debug("Shooter Actual RPM", shooterMotor.getVelocity());
        panelsTelemetry.debug("Turret Target Ticks", getCurrentTurretTicks());
        panelsTelemetry.debug("Turret Current Ticks", turretMotor.getCurrentPosition());
        panelsTelemetry.debug("Turret Reached", turretReachedTarget);
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain preLoadShot;
        public PathChain pickup1;
        public PathChain shoot1;
        public PathChain pickup2;
        public PathChain shoot2;
        public PathChain pickup3;
        public PathChain shoot3;

        public Paths(Follower follower) {
            preLoadShot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.000, 134.500),
                                    new Pose(65.000, 73.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(65.000, 73.000),
                                    new Pose(54.261, 50.717),
                                    new Pose(18.500, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.100, 82.000),
                                    new Pose(67.000, 73.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(67.000, 73.000),
                                    new Pose(50.188, 20.940),
                                    new Pose(18.500, 37.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.500, 60.000),
                                    new Pose(64.000, 73.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            pickup3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(64.000, 73.000),
                                    new Pose(44.500, 83.500),
                                    new Pose(24.100, 82.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.500, 37.000),
                                    new Pose(60.000, 110.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    /**
     * Get the shooter RPM for the current shot
     */
    private double getCurrentShooterRPM() {
        if (currentShotIndex < SHOT_RPM.length) {
            return SHOT_RPM[currentShotIndex];
        }
        return SHOT_RPM[0]; // Default to first value if out of bounds
    }

    /**
     * Get the turret tick target for the current shot
     */
    private int getCurrentTurretTicks() {
        if (currentShotIndex < TURRET_TICKS.length) {
            return TURRET_TICKS[currentShotIndex];
        }
        return TURRET_TICKS[0]; // Default to first value if out of bounds
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            // ========== PRELOAD SHOT CYCLE ==========
            case 0: // preLoadShot path - Start shooter and turret
                // Start shooter motor at RPM for this shot
                shooterMotor.setVelocity(getCurrentShooterRPM());

                // Start turret motor rotation to position for this shot
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }

                // Follow the path
                follow(paths.preLoadShot);

                // Advance when path is complete
                advanceAfterPath();
                break;

            case 1: // Shooting sequence - stopper and intake
                // Shooter motor stays on from case 0 at current RPM

                // Open turret stopper after 1.5 seconds
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);

                    // Run intake (negative to push ball out)
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                }

                // Advance after enough time for shooting
                advanceAfter(2.7); // Total time for stopper + intake
                break;

            case 2: // Reset after preload shot
                // Close turret stopper
                turretServo.setPosition(TURRET_HOME);

                // Stop intake
                intakeMotor.setVelocity(0);

                // Keep shooter running for next shot
                // Reset turret motor for next movement
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;

                currentShotIndex++; // Move to next shot configuration
                advance();
                break;

            // ========== PICKUP 1 ==========
            case 3: // Move to pickup position 1 and intake balls
                // Turn on intake to pick up balls (positive to intake)
                intakeMotor.setVelocity(-INTAKE_VELOCITY);

                // Shooter keeps running in background

                // Follow pickup path
                follow(paths.pickup3);

                // Advance when path is complete
                advanceAfterPath();
                break;

            case 4: // Stop intake after pickup
                // Stop intake
                intakeMotor.setVelocity(0);

                advance();
                break;

            // ========== SHOOT 1 CYCLE ==========
            case 5: // Move to shooting position with new shooter/turret settings
                // Set shooter to new RPM for shot 1
                shooterMotor.setVelocity(getCurrentShooterRPM());

                // Start turret motor rotation to new position for shot 1
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }

                // Follow path back to shooting position
                follow(paths.shoot1);

                // Advance when path is complete
                advanceAfterPath();
                break;

            case 6: // Execute shot 1
                // Shooter motor stays on from case 5 at new RPM

                // Open turret stopper after 1.5 seconds
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);

                    // Run intake (negative to push ball out)
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                }

                // Advance after enough time for shooting
                advanceAfter(2.7);
                break;

            case 7: // Reset after shot 1
                // Close turret stopper
                turretServo.setPosition(TURRET_HOME);

                // Stop intake
                intakeMotor.setVelocity(0);

                // Keep shooter running for next shot
                // Reset turret motor for next movement
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;

                currentShotIndex++; // Move to shot 2 configuration
                advance();
                break;

            // ========== PICKUP 2 ==========
            case 8: // Move to pickup position 2 and intake balls
                // Turn on intake to pick up balls
                intakeMotor.setVelocity(-INTAKE_VELOCITY);

                // Shooter keeps running in background

                // Follow pickup path
                follow(paths.pickup1);

                // Advance when path is complete
                advanceAfterPath();
                break;

            case 9: // Stop intake after pickup
                // Stop intake
                intakeMotor.setVelocity(0);

                advance();
                break;

            // ========== SHOOT 2 CYCLE ==========
            case 10: // Move to shooting position with new shooter/turret settings
                // Set shooter to new RPM for shot 2
                shooterMotor.setVelocity(getCurrentShooterRPM());

                // Start turret motor rotation to new position for shot 2
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }

                // Follow path back to shooting position
                follow(paths.shoot2);

                // Advance when path is complete
                advanceAfterPath();
                break;

            case 11: // Execute shot 2
                // Shooter motor stays on from case 10 at new RPM

                // Open turret stopper after 1.5 seconds
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);

                    // Run intake (negative to push ball out)
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                }

                // Advance after enough time for shooting
                advanceAfter(2.7);
                break;

            case 12: // Reset after shot 2
                // Close turret stopper
                turretServo.setPosition(TURRET_HOME);

                // Stop intake
                intakeMotor.setVelocity(0);

                // Keep shooter running for next shot
                // Reset turret motor for next movement
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;

                currentShotIndex++; // Move to shot 3 configuration
                advance();
                break;

            // ========== PICKUP 3 ==========
            case 13: // Move to pickup position 3 and intake balls
                // Turn on intake to pick up balls
                intakeMotor.setVelocity(-INTAKE_VELOCITY);

                // Shooter keeps running in background

                // Follow pickup path
                follow(paths.pickup2);

                // Advance when path is complete
                advanceAfterPath();
                break;

            case 14: // Stop intake after pickup
                // Stop intake
                intakeMotor.setVelocity(0);

                advance();
                break;

            // ========== SHOOT 3 CYCLE (FINAL) ==========
            case 15: // Move to shooting position with new shooter/turret settings
                // Set shooter to new RPM for shot 3
                shooterMotor.setVelocity(getCurrentShooterRPM());

                // Start turret motor rotation to new position for shot 3
                if (!turretMotorActive) {
                    turretMotorActive = true;
                    turretReachedTarget = false;
                }

                // Follow path back to shooting position
                follow(paths.shoot3);

                // Advance when path is complete
                advanceAfterPath();
                break;

            case 16: // Execute shot 3 (final shot)
                // Shooter motor stays on from case 15 at new RPM

                // Open turret stopper after 1.5 seconds
                if (StateTimer.seconds() >= STOPPER_OPEN_TIME) {
                    turretServo.setPosition(TURRET_ACTIVE);

                    // Run intake (negative to push ball out)
                    intakeMotor.setVelocity(-INTAKE_VELOCITY);
                }

                // Advance after enough time for shooting
                advanceAfter(2.7);
                break;

            case 17: // FINAL - Stop everything
                // Close turret stopper
                turretServo.setPosition(TURRET_HOME);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotorActive = false;
                turretReachedTarget = false;

                currentShotIndex++; // Move to shot 2 configuration
                // Stop intake
                intakeMotor.setVelocity(0);

                // Stop shooter
                shooterMotor.setVelocity(0);

                // Stop turret motor
                turretMotor.setPower(0);

                // Program complete - all paths executed
                break;

            default:
                // Safety stop
                shooterMotor.setVelocity(0);
                intakeMotor.setVelocity(0);
                turretMotor.setPower(0);
                break;
        }
        return pathState;
    }

    /**
     * Updates turret motor to move to target position during path following
     * Uses a holding power to prevent springback after reaching target
     */
    private void updateTurretMotor() {
        if (turretMotorActive && !turretReachedTarget) {
            int currentTicks = turretMotor.getCurrentPosition();
            int targetTicks = getCurrentTurretTicks();
            int error = targetTicks - currentTicks;

            // Check if target reached (with small tolerance)
            if (Math.abs(error) < TURRET_POSITION_TOLERANCE) {
                turretReachedTarget = true;
            } else {
                // Move toward target with proportional power for smoother approach
                double power = TURRET_POWER;

                // Slow down as we approach target (proportional control)
                if (Math.abs(error) < 100) {
                    power = 0.3 + (Math.abs(error) / 100.0) * (TURRET_POWER - 0.3);
                }

                if (error > 0) {
                    turretMotor.setPower(power);
                } else {
                    turretMotor.setPower(-power);
                }
            }
        }

        // Hold position when target reached to prevent springback
        if (turretReachedTarget && turretMotorActive) {
            int currentTicks = turretMotor.getCurrentPosition();
            int targetTicks = getCurrentTurretTicks();
            int error = targetTicks - currentTicks;

            // Use a small holding power to maintain position
            double holdPower = error * TURRET_HOLD_kP; // Proportional holding
            holdPower = Math.max(-TURRET_HOLD_POWER_MAX, Math.min(TURRET_HOLD_POWER_MAX, holdPower)); // Limit holding power

            turretMotor.setPower(holdPower);
        }

        if (!turretMotorActive) {
            turretMotor.setPower(0);
        }
    }

    private void follow(PathChain path) {
        follow(path, false);
    }

    public void follow(PathChain path, boolean holdPoint) {
        if (pathState != lastState) {
            follower.followPath(path, holdPoint);
            lastState = pathState;
        }
    }

    private void advance() {
        pathState++;
        StateTimer.reset();
        isWaiting = false;
    }

    private void advanceWhen(boolean cond) {
        if (cond) advance();
    }

    private void advanceAfterPath() {
        advanceWhen(!follower.isBusy());
    }

    private void advanceAfterPathWithMin(double minimumTime) {
        if (StateTimer.seconds() > minimumTime)
            advanceAfterPath();
    }

    private void advanceAfter(double seconds) {
        if (!isWaiting) advanceTimer.reset();
        isWaiting = true;
        advanceWhen(advanceTimer.seconds() > seconds);
        if (advanceTimer.seconds() > seconds)
            isWaiting = false;
    }
}