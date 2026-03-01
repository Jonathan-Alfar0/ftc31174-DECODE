package org.firstinspires.ftc.teamcode.Regionals;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathPoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.teleConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.TurretAiming;

@TeleOp(name = "BLUE FAR Tele (2) - Regionals", group = "0_Primary")
public class FarTurretAndShooter extends LinearOpMode {

    // ==================== HARDWARE ====================
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx intakeMotor;
    private IMU imu;
    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private Servo hoodServo;
    private Servo turretStopperServo;
    private Follower follower;

    // ==================== CONTROLLERS ====================
    private TurretAiming turretAiming;

    // ==================== DRIVE STATE ====================
    private double maxSpeed = 0.93;

    // ==================== SHOOTER STATE & PIDF ====================
    private boolean isAutomationActive = false;
    private boolean isShooterOn = false;
    private double targetRPM = 0;
    private double hoodPosition = 0;
    private double shooter_kP = 180.0;
    private double shooter_kI = 0.0;
    private double shooter_kD = 1.0;
    private double shooter_kF = 19.1;
    private final double TICKS_PER_REV = 28;
    private static final double INTAKE_VELOCITY = 1800;
    private TurretAiming.Alliance currentAlliance = TurretAiming.Alliance.BLUE;

    // ==================== TURRET STOPPER STATE ====================
    private boolean isTurretStopperActive = false;
    private long turretStopperStartTime = 0;
    private static final double TURRET_STOPPER_HOME = 0.45;
    private static final double TURRET_STOPPER_ACTIVE = 0.2;
    private static final long TURRET_STOPPER_TIME_MS = 1200;

    // ==================== GATE AUTOMATION ====================

    /**
     * GATE_POSE_1 — robot position/heading to trigger the gate
     * GATE_POSE_2 — robot position/heading immediately after passing through
     * TODO: tune x, y, heading to your actual field positions
     */
    private static final Pose GATE_POSE_1 = new Pose(17.0886, 61.5776, Math.toRadians(155));
    private static final Pose GATE_POSE_2 = new Pose(15.9939, 52.771,  Math.toRadians(154));
    private enum GateState { IDLE, TO_GATE, THROUGH_GATE }
    private GateState gateState = GateState.IDLE;

    // ==================== PARK AUTOMATION (GP1 Right Trigger hold) ====================

    /**
     * PARK_POSE — the field position the robot drives to when parking.
     * TODO: replace x, y, and heading with your actual desired park position.
     *
     * Example: somewhere safe in your alliance zone, facing forward —
     *   new Pose(12.0, 12.0, Math.toRadians(0))
     */
    private static final Pose PARK_POSE = new Pose(
            0,              // <-- TODO: replace with your park X (inches)
            0,              // <-- TODO: replace with your park Y (inches)
            Math.toRadians(0)  // <-- TODO: replace with your park heading (degrees)
    );

    private enum ParkState { IDLE, PARKING }
    private ParkState parkState = ParkState.IDLE;

    // ==================== MANUAL POSE RESET (GP2 X) ====================

    /**
     * TODO: Set this to the known field position the robot will be at
     *       when the driver presses X to correct odometry drift.
     *       x, y in inches, heading in degrees (converted below).
     *
     * Example: robot is backed against the wall near the far side —
     *   new Pose(92.8018, 19.9742, Math.toRadians(0))
     */
    private static final Pose MANUAL_RESET_POSE = new Pose(
            0,           // <-- TODO: replace with your desired X (inches)
            0,           // <-- TODO: replace with your desired Y (inches)
            Math.toRadians(0)  // <-- TODO: replace with your desired heading (degrees)
    );

    /**
     * IDLE       — normal operation
     * RESETTING  — pose was just applied, turret driving to zero
     * ZEROED     — turret confirmed at zero, ready to resume automation
     */
    private enum PoseResetState { IDLE, RESETTING, ZEROED }
    private PoseResetState poseResetState = PoseResetState.IDLE;

    // Tracks the pose that was last applied so telemetry can confirm it
    private Pose lastAppliedResetPose = null;

    // ==================== BUTTON DEBOUNCING ====================
    private boolean lastGp1RightTrigger = false;
    private boolean lastLeftBumper   = false;
    private boolean lastYButton      = false;
    private boolean lastRightTrigger = false;
    private boolean lastDPadUp       = false;
    private boolean lastDPadDown     = false;
    private boolean lastXButton      = false;

    @Override
    public void runOpMode() {
        initializeHardware();

        turretAiming = new TurretAiming(turretMotor, null);
        turretAiming.setAlliance(TurretAiming.Alliance.BLUE);
        turretAiming.setVelocityCompensation(true);
        turretAiming.disableLimelightCorrection();

        telemetry.addLine("### RED FAR Tele - Regionals ###");
        telemetry.addLine("GP1 Hold R_Trigger = Auto Park");
        telemetry.addLine("GP2 X         = Manual Pose Reset (drift fix)");
        telemetry.addLine("GP2 L_Trigger = Gate Auto");
        telemetry.addLine("GP2 L_Bumper  = Turret Aim Toggle");
        telemetry.addLine("Ready to Start!");
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            follower.update();
            Pose robotPose = follower.getPose();

            handleManualPoseReset(robotPose); // Must run before automation
            handleGateAutomation(robotPose);
            handlePark(robotPose);             // GP1 right trigger — runs before handleDrive
            handleDrive();                     // Respects parkState and gateState
            handleIntake();
            handleAutomation(robotPose);
            displayTelemetry(robotPose);
        }

        shutdownRobot();
    }

    // ==================== MANUAL POSE RESET (GP2 X) ====================

    /**
     * Single press of GP2 X:
     *   1. Immediately snaps follower pose to MANUAL_RESET_POSE
     *   2. Sends turret to zero (goHome)
     *   3. Once turret reports zeroed → state becomes ZEROED
     *   4. Driver can now press L_Bumper to enable automation —
     *      the follower already has the corrected pose so turret aims correctly
     *
     * A second X press at any point cancels back to IDLE.
     */
    private void handleManualPoseReset(Pose robotPose) {
        boolean xPressed = gamepad2.x && !lastXButton;
        lastXButton = gamepad2.x;

        switch (poseResetState) {

            case IDLE:
                if (xPressed) {
                    // ── Apply the pose correction immediately ──
                    follower.setPose(MANUAL_RESET_POSE);
                    lastAppliedResetPose = MANUAL_RESET_POSE;

                    // ── Pause automation and send turret home ──
                    isAutomationActive = false;
                    isShooterOn = false;
                    turretAiming.stopAiming();
                    turretAiming.goHome();

                    poseResetState = PoseResetState.RESETTING;
                }
                break;

            case RESETTING:
                // Keep driving turret home until zeroed
                if (!turretAiming.isAiming()) turretAiming.goHome();

                if (turretAiming.isZeroed()) {
                    poseResetState = PoseResetState.ZEROED;
                }

                // Cancel with second X press
                if (xPressed) {
                    poseResetState = PoseResetState.IDLE;
                }
                break;

            case ZEROED:
                // Sit here until the driver enables automation via L_Bumper
                // (handleAutomation handles that toggle — we just watch)
                // Auto-clear once automation is re-enabled
                if (isAutomationActive) {
                    poseResetState = PoseResetState.IDLE;
                }
                // Allow re-triggering X to run another reset
                if (xPressed) {
                    follower.setPose(MANUAL_RESET_POSE);
                    lastAppliedResetPose = MANUAL_RESET_POSE;
                    turretAiming.goHome();
                    poseResetState = PoseResetState.RESETTING;
                }
                break;
        }
    }

    // ==================== GATE AUTOMATION (GP2 Left Trigger hold) ====================

    private void handleGateAutomation(Pose robotPose) {
        boolean buttonHeld = gamepad1.left_bumper;

        switch (gateState) {

            case IDLE:
                if (buttonHeld && !lastLeftBumper) {
                    PathChain leg1 = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(robotPose, GATE_POSE_1)))
                            .setLinearHeadingInterpolation(
                                    robotPose.getHeading(),
                                    GATE_POSE_1.getHeading())
                            .build();
                    follower.followPath(leg1, true);
                    gateState = GateState.TO_GATE;
                }
                break;

            case TO_GATE:
                if (!buttonHeld) { cancelGate(); break; }
                if (!follower.isBusy()) {
                    PathChain leg2 = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(GATE_POSE_1, GATE_POSE_2)))
                            .setLinearHeadingInterpolation(
                                    GATE_POSE_1.getHeading(),
                                    GATE_POSE_2.getHeading())
                            .build();
                    follower.followPath(leg2, true);
                    gateState = GateState.THROUGH_GATE;
                }
                break;

            case THROUGH_GATE:
                if (!buttonHeld || !follower.isBusy()) cancelGate();
                break;
        }

        lastLeftBumper = buttonHeld;
    }

    private void cancelGate() {
        follower.startTeleopDrive();
        gateState = GateState.IDLE;
    }

    // ==================== PARK AUTOMATION (GP1 Right Trigger hold) ====================

    /**
     * Hold GP1 right trigger to autonomously drive to PARK_POSE.
     * Release at any time to cancel and return to teleop drive.
     * Path is rebuilt fresh from the current robot pose every time it's triggered,
     * so it works from wherever the robot happens to be on the field.
     */
    private void handlePark(Pose robotPose) {
        boolean triggerHeld = gamepad1.left_trigger > 0.5;

        switch (parkState) {

            case IDLE:
                if (triggerHeld && !lastGp1RightTrigger) {
                    // Build a straight-line path from current pose to the park pose
                    PathChain parkPath = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(robotPose, PARK_POSE)))
                            .setLinearHeadingInterpolation(
                                    robotPose.getHeading(),
                                    PARK_POSE.getHeading())
                            .build();
                    follower.followPath(parkPath, true);
                    parkState = ParkState.PARKING;
                }
                break;

            case PARKING:
                // Release trigger → cancel immediately
                if (!triggerHeld) {
                    cancelPark();
                    break;
                }
                // Path finished → stay braked at park pose, return to idle
                if (!follower.isBusy()) {
                    parkState = ParkState.IDLE;
                    // Leave follower in path-following mode so it holds position;
                    // driver can nudge off with stick if needed
                    follower.startTeleopDrive();
                }
                break;
        }

        lastGp1RightTrigger = triggerHeld;
    }

    private void cancelPark() {
        follower.startTeleopDrive();
        parkState = ParkState.IDLE;
    }

    // ==================== DRIVE ====================

    private void handleDrive() {
        if (gateState != GateState.IDLE) return;
        if (parkState != ParkState.IDLE)  return;

        if (gamepad1.a) maxSpeed = 0.55;
        if (gamepad1.b) maxSpeed = 0.93;

        double forward = -gamepad1.left_stick_y * maxSpeed;
        double right   =  gamepad1.left_stick_x * maxSpeed;
        double rotate  =  gamepad1.right_stick_x * maxSpeed;

        frontLeftDrive.setPower(forward + right + rotate);
        frontRightDrive.setPower(forward - right - rotate);
        backLeftDrive.setPower(forward - right + rotate);
        backRightDrive.setPower(forward + right - rotate);
    }

    // ==================== AUTOMATION (TURRET / SHOOTER) ====================

    private void handleAutomation(Pose robotPose) {
        // Alliance Selection
        if (gamepad2.a) {
            turretAiming.setAlliance(TurretAiming.Alliance.BLUE);
            currentAlliance = TurretAiming.Alliance.BLUE;
        }
        if (gamepad2.b) {
            turretAiming.setAlliance(TurretAiming.Alliance.RED);
            currentAlliance = TurretAiming.Alliance.RED;
        }

        // Turret automation toggle (left bumper) — blocked while gate is running
        // Also blocked while pose is RESETTING so we don't fight goHome()
        if (gateState == GateState.IDLE
                && poseResetState != PoseResetState.RESETTING
                && gamepad2.left_bumper && !lastLeftBumper) {
            isAutomationActive = !isAutomationActive;
            if (!isAutomationActive) isShooterOn = false;
        }

        // Shooter toggle (right trigger)
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;
        if (isAutomationActive && rightTriggerPressed && !lastRightTrigger) {
            isShooterOn = !isShooterOn;
        }
        lastRightTrigger = rightTriggerPressed;

        // Turret stopper (right bumper)
        if (gamepad2.right_bumper && !isTurretStopperActive) {
            turretStopperServo.setPosition(TURRET_STOPPER_ACTIVE);
            turretStopperStartTime = System.currentTimeMillis();
            isTurretStopperActive = true;
        }
        if (isTurretStopperActive
                && System.currentTimeMillis() - turretStopperStartTime >= TURRET_STOPPER_TIME_MS) {
            turretStopperServo.setPosition(TURRET_STOPPER_HOME);
            isTurretStopperActive = false;
        }

        // Aiming state sync — don't fight goHome() while resetting
        if (poseResetState != PoseResetState.RESETTING) {
            if (isAutomationActive && !turretAiming.isAiming()) turretAiming.startAiming();
            else if (!isAutomationActive && turretAiming.isAiming()) turretAiming.stopAiming();
        }

        if (isAutomationActive) {
            if (gamepad2.dpad_up   && !lastDPadUp)   turretAiming.enableLimelightCorrection();
            if (gamepad2.dpad_down && !lastDPadDown)  turretAiming.disableLimelightCorrection();
            if (gamepad2.y         && !lastYButton)   turretAiming.toggleVelocityCompensation();
            if (gamepad2.back) { isAutomationActive = false; isShooterOn = false; }
        }
        lastDPadUp   = gamepad2.dpad_up;
        lastDPadDown = gamepad2.dpad_down;
        lastYButton  = gamepad2.y;

        // Always update turret — handles aiming and goHome internally
        turretAiming.update(robotPose, robotPose.getHeading());

        if (isAutomationActive) {
            double distance = Math.hypot(
                    turretAiming.getCurrentGoal().getX() - robotPose.getX(),
                    turretAiming.getCurrentGoal().getY() - robotPose.getY());
            targetRPM    = ShooterConstants.targetRPM(distance);
            hoodPosition = ShooterConstants.hoodPosition(distance);
            hoodServo.setPosition(hoodPosition);

            if (isShooterOn) setShooterVelocity(targetRPM);
            else { shooterMotor1.setPower(0); shooterMotor2.setPower(0); }
        } else {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
        }
    }

    // ==================== INTAKE ====================

    private void handleIntake() {
        // Intake is disabled while auto-parking so the trigger doesn't conflict
        if (parkState != ParkState.IDLE) {
            intakeMotor.setVelocity(0);
            return;
        }
        if (gamepad1.right_trigger > 0.1) {
            intakeMotor.setVelocity(INTAKE_VELOCITY);
        } else if (gamepad1.right_bumper) {
            intakeMotor.setVelocity(-INTAKE_VELOCITY);
        } else {
            intakeMotor.setVelocity(0);
        }
    }

    // ==================== SHOOTER ====================

    private void setShooterVelocity(double rpm) {
        double targetTicksPerSec = rpm * TICKS_PER_REV / 60.0;
        shooterMotor1.setVelocity(targetTicksPerSec);
        shooterMotor2.setVelocity(targetTicksPerSec);
    }

    // ==================== INIT ====================

    private void initializeHardware() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter_motor2");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor1.setVelocityPIDFCoefficients(shooter_kP, shooter_kI, shooter_kD, shooter_kF);
        shooterMotor2.setVelocityPIDFCoefficients(shooter_kP, shooter_kI, shooter_kD, shooter_kF);

        hoodServo = hardwareMap.get(Servo.class, "turret_servo");
        hoodServo.setDirection(Servo.Direction.REVERSE);

        turretStopperServo = hardwareMap.get(Servo.class, "turretStopper");
        turretStopperServo.setDirection(Servo.Direction.REVERSE);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretStopperServo.setPosition(TURRET_STOPPER_HOME);

        imu = hardwareMap.get(IMU.class, "imu");

        follower = teleConstants.createFollower(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(51.1982, 19.9742, Math.toRadians(180)));
    }

    // ==================== SHUTDOWN ====================

    private void shutdownRobot() {
        turretAiming.stopAiming();
        shooterMotor1.setPower(0); shooterMotor2.setPower(0);
        intakeMotor.setPower(0);
        frontLeftDrive.setPower(0);  frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);   backRightDrive.setPower(0);
    }

    // ==================== TELEMETRY ====================

    private void displayTelemetry(Pose robotPose) {
        double distance = Math.hypot(
                turretAiming.getCurrentGoal().getX() - robotPose.getX(),
                turretAiming.getCurrentGoal().getY() - robotPose.getY());
        double currentRPM1 = shooterMotor1.getVelocity() * 60 / TICKS_PER_REV;

        telemetry.addLine("--- GP1: Drive/Intake | GP2: Operate ---");
        telemetry.addData("Turret Auto (GP2 L_Bumper)", isAutomationActive ? "ON" : "OFF");
        telemetry.addData("Shooter (GP2 R_Trigger)",    isShooterOn ? "ON" : "OFF");
        telemetry.addData("Intake Vel", "%.1f", intakeMotor.getVelocity());
        telemetry.addLine();

        // ── Manual Pose Reset Telemetry ──────────────────────────────────────
        telemetry.addLine("--- Manual Pose Reset (GP2 X) ---");
        telemetry.addData("Reset State", poseResetState.name());

        // Show what the reset pose is configured to
        telemetry.addData("Reset Pose (target)",
                "(%.2f, %.2f, %.1f°)",
                MANUAL_RESET_POSE.getX(),
                MANUAL_RESET_POSE.getY(),
                Math.toDegrees(MANUAL_RESET_POSE.getHeading()));

        // Show the actual follower pose live so the driver can confirm the snap happened
        telemetry.addData("Follower Pose (live)",
                "(%.2f, %.2f, %.1f°)",
                robotPose.getX(),
                robotPose.getY(),
                Math.toDegrees(robotPose.getHeading()));

        // Confirm the last applied reset
        if (lastAppliedResetPose != null) {
            telemetry.addData("Last Applied Reset",
                    "(%.2f, %.2f, %.1f°)",
                    lastAppliedResetPose.getX(),
                    lastAppliedResetPose.getY(),
                    Math.toDegrees(lastAppliedResetPose.getHeading()));
        } else {
            telemetry.addData("Last Applied Reset", "None yet");
        }

        // Guide the driver through the reset sequence
        switch (poseResetState) {
            case IDLE:
                telemetry.addData("Action", "Press X to reset pose & home turret");
                break;
            case RESETTING:
                telemetry.addData("Action", "Turret homing... wait for ZEROED");
                telemetry.addData("Turret Zeroed", turretAiming.isZeroed() ? "YES ✓" : "NO - moving...");
                telemetry.addData("Turret Ticks (raw)", turretAiming.getCurrentTicks());
                break;
            case ZEROED:
                telemetry.addData("Action", "Pose set + turret zeroed! Press L_Bumper to aim");
                telemetry.addData("Turret Zeroed", "YES ✓");
                break;
        }
        telemetry.addLine();

        // ── Gate Telemetry ───────────────────────────────────────────────────
        telemetry.addLine("--- Gate (GP2 Hold L_Trigger) ---");
        telemetry.addData("Gate State", gateState.name());
        PathPoint desiredPoint = follower.getClosestPose();
        if (desiredPoint != null && desiredPoint.getPose() != null) {
            Pose desiredPose = desiredPoint.getPose();
            telemetry.addData("Desired Pose", "(%.1f, %.1f, %.1f°)",
                    desiredPose.getX(), desiredPose.getY(), Math.toDegrees(desiredPose.getHeading()));
        } else {
            Pose targetPose = gateState == GateState.TO_GATE ? GATE_POSE_1
                    : gateState == GateState.THROUGH_GATE ? GATE_POSE_2
                    : null;
            if (targetPose != null) {
                telemetry.addData("Target Pose", "(%.1f, %.1f, %.1f°)",
                        targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading()));
            } else {
                telemetry.addData("Target Pose", "N/A (IDLE)");
            }
        }
        telemetry.addLine();

        telemetry.addLine();

        // ── Park Telemetry ───────────────────────────────────────────────────
        telemetry.addLine("--- Auto Park (GP1 Hold R_Trigger) ---");
        telemetry.addData("Park State", parkState.name());
        telemetry.addData("Park Target",
                "(%.2f, %.2f, %.1f°)",
                PARK_POSE.getX(),
                PARK_POSE.getY(),
                Math.toDegrees(PARK_POSE.getHeading()));

        if (parkState == ParkState.PARKING) {
            double distToPark = Math.hypot(
                    PARK_POSE.getX() - robotPose.getX(),
                    PARK_POSE.getY() - robotPose.getY());
            telemetry.addData("Dist to Park", "%.1f in", distToPark);
            telemetry.addData("Action", "PARKING — release trigger to cancel");
        } else {
            telemetry.addData("Action", "Hold GP1 R_Trigger to park");
        }

        // ── Aiming Controls Telemetry ────────────────────────────────────────
        telemetry.addLine("--- GP2 Aiming Controls ---");
        telemetry.addData("Alliance (A/B)",             currentAlliance);
        telemetry.addData("Limelight Aim (D-Up/D-Down)", turretAiming.isUsingLimelight() ? "ON" : "OFF");
        telemetry.addData("Vel Comp (Y)",               turretAiming.isUsingVelocityCompensation() ? "ON" : "OFF");
        telemetry.addData("Turret Status",              turretAiming.getStatusString());
        telemetry.addLine();

        // ── Turret & Shooter Telemetry ───────────────────────────────────────
        telemetry.addLine("--- Turret & Shooter ---");
        telemetry.addData("Distance",      "%.1f in", distance);
        telemetry.addData("Goal X/Y",
                "(%.2f, %.2f)",
                turretAiming.getCurrentGoal().getX(),
                turretAiming.getCurrentGoal().getY());

        telemetry.addData("Turret Ticks (raw)", turretAiming.getCurrentTicks());
        telemetry.addData("Turret Zeroed",      turretAiming.isZeroed() ? "YES" : "NO");
        telemetry.addData("Target RPM",  "%.1f", targetRPM);
        telemetry.addData("Actual RPM",  "%.1f", currentRPM1);
        telemetry.addData("Hood Target", "%.3f", hoodPosition);
        telemetry.addData("Hood Actual", "%.3f", hoodServo.getPosition());
        telemetry.addData("Stopper State", isTurretStopperActive ? "ACTIVE" : "HOME");
        telemetry.addData("Stopper Pos",   "%.3f", turretStopperServo.getPosition());
        telemetry.update();
    }
}