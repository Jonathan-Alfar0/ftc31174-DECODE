package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.teleConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.TurretAiming;

/**
 * Championship TeleOp
 *
 * Integrates odometry-based turret aiming, distance-based shooter RPM/hood lookup,
 * and full driver controls.
 *
 * ─────────────────────────────────────────
 * GAMEPAD 1 (DRIVER):
 *   Left Stick:      Drive (strafe + forward)
 *   Right Stick X:   Rotate
 *   Right Trigger:   Intake IN
 *   Right Bumper:    Intake OUT
 *
 * GAMEPAD 2 (OPERATOR):
 *   D-Pad UP:        Set BLUE alliance goal
 *   D-Pad DOWN:      Set RED alliance goal
 *   Left Bumper:     Toggle localizer aiming ON/OFF
 *   Right Bumper:    Toggle localizer mode (ODOMETRY <-> LIMELIGHT)
 *   Right Trigger:   Toggle shooter flywheels ON/OFF
 *   X:               Return turret to home (0 ticks)
 *   A:               Activate turretStopper servo
 * ─────────────────────────────────────────
 */
@TeleOp(name = "CHAMPIONSHIP - TeleOp", group = "COMPETITION")
public class ChampionshipTeleOp extends LinearOpMode {

    // ==================== HARDWARE ====================
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private DcMotorEx intakeMotor;
    private Servo turretStopperServo;   // "turretStopper" - the ball release servo
    private Servo hoodServo;            // "turret_servo"  - adjusts shooter hood angle
    private Limelight3A limelight;
    private Follower follower;

    // ==================== TURRET AIMING ====================
    private TurretAiming turretAiming;

    // ==================== SHOOTER CONSTANTS ====================
    private static final double SHOOTER_kP = 180.0;
    private static final double SHOOTER_kI = 13.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kF = 18.0;

    // ==================== INTAKE CONSTANTS ====================
    private static final double INTAKE_VELOCITY = 1800;

    // ==================== TURRET STOPPER SERVO ====================
    private static final double TURRET_STOPPER_HOME   = 0.45;
    private static final double TURRET_STOPPER_ACTIVE = 0.20;
    private static final long   TURRET_STOPPER_TIME_MS = 1500;

    // ==================== LIMELIGHT DISTANCE CONSTANTS ====================
    /** Height of camera lens from floor (inches) — tune for your robot */
    private static final double CAMERA_HEIGHT_INCHES      = 12.0;
    /** Height of the goal target center from floor (inches) — tune for your field */
    private static final double TARGET_HEIGHT_INCHES      = 29.5;
    /** Camera mount angle in degrees (0 = perfectly horizontal) */
    private static final double CAMERA_MOUNT_ANGLE_DEG    = 0.0;

    // ==================== STATE ====================
    private double  maxSpeed     = 0.93;
    private boolean shooterOn    = false;

    // Turret stopper servo timing
    private boolean turretStopperActive   = false;
    private long    turretStopperStartMs  = 0;

    // Button debouncing
    private boolean lastLeftBumper2   = false;
    private boolean lastRightBumper2  = false;
    private boolean lastRightTrigger2 = false;
    private boolean lastDpadUp2       = false;
    private boolean lastDpadDown2     = false;
    private boolean lastX2            = false;
    private boolean lastA2            = false;

    // Localizer mode
    private boolean useLimelight = false;  // false = odometry, true = limelight

    @Override
    public void runOpMode() {

        // ── Hardware init ──
        initHardware();

        // ── Turret aiming controller ──
        turretAiming = new TurretAiming(turretMotor, limelight);
        turretAiming.setAlliance(TurretAiming.Alliance.BLUE);  // default alliance

        // ── Shooter PIDF ──
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setVelocityPIDFCoefficients(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kF);

        // ── Stopper servo home ──
        turretStopperServo.setPosition(TURRET_STOPPER_HOME);

        telemetry.addLine("══════════════════════════════════");
        telemetry.addLine("      CHAMPIONSHIP TELEOP READY   ");
        telemetry.addLine("══════════════════════════════════");
        telemetry.addLine("GP2 D-UP: Blue  |  D-DOWN: Red");
        telemetry.addLine("GP2 L-BUMP: Toggle Aim");
        telemetry.addLine("GP2 R-BUMP: Toggle Odom/Limelight");
        telemetry.addLine("GP2 R-TRIG: Toggle Shooter");
        telemetry.addLine("GP2 X: Home Turret  |  A: Release");
        telemetry.addLine("Press START to begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ═══════════════════════════════════════════
            //  ODOMETRY UPDATE
            // ═══════════════════════════════════════════
            follower.update();
            Pose robotPose   = follower.getPose();
            double robotHeading = robotPose.getHeading();

            // ═══════════════════════════════════════════
            //  DISTANCE CALCULATION
            //  Primary: odometry distance to goal
            //  Backup:  limelight trig if valid & in limelight mode
            // ═══════════════════════════════════════════
            Pose   goal      = turretAiming.getCurrentGoal();
            double dx        = goal.getX() - robotPose.getX();
            double dy        = goal.getY() - robotPose.getY();
            double odomDist  = Math.sqrt(dx * dx + dy * dy);

            double distance  = odomDist;  // start with odometry

            LLResult llResult = (limelight != null) ? limelight.getLatestResult() : null;
            double   llDist   = -1;

            if (llResult != null && llResult.isValid()) {
                double ty = llResult.getTy();
                double trigDist = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES)
                        / Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + ty));
                if (trigDist > 0) {
                    llDist = trigDist;
                    // In limelight mode, use limelight distance instead
                    if (useLimelight) {
                        distance = llDist;
                    }
                }
            }

            // ═══════════════════════════════════════════
            //  GAMEPAD 2 — OPERATOR CONTROLS
            // ═══════════════════════════════════════════

            // D-Pad UP: Blue alliance
            if (gamepad2.dpad_up && !lastDpadUp2) {
                turretAiming.setAlliance(TurretAiming.Alliance.BLUE);
                telemetry.speak("Blue alliance");
            }
            lastDpadUp2 = gamepad2.dpad_up;

            // D-Pad DOWN: Red alliance
            if (gamepad2.dpad_down && !lastDpadDown2) {
                turretAiming.setAlliance(TurretAiming.Alliance.RED);
                telemetry.speak("Red alliance");
            }
            lastDpadDown2 = gamepad2.dpad_down;

            // Left Bumper: Toggle aiming on/off
            if (gamepad2.left_bumper && !lastLeftBumper2) {
                turretAiming.toggleAiming();
                telemetry.speak(turretAiming.isAiming() ? "Aiming on" : "Aiming off");
            }
            lastLeftBumper2 = gamepad2.left_bumper;

            // Right Bumper: Toggle odometry vs limelight localizer
            if (gamepad2.right_bumper && !lastRightBumper2) {
                useLimelight = !useLimelight;
                if (useLimelight) {
                    turretAiming.enableLimelightCorrection();
                    telemetry.speak("Limelight mode");
                } else {
                    turretAiming.disableLimelightCorrection();
                    telemetry.speak("Odometry mode");
                }
            }
            lastRightBumper2 = gamepad2.right_bumper;

            // Right Trigger: Toggle shooter flywheels
            boolean triggerPressed2 = gamepad2.right_trigger > 0.5;
            if (triggerPressed2 && !lastRightTrigger2) {
                shooterOn = !shooterOn;
                telemetry.speak(shooterOn ? "Shooter on" : "Shooter off");
            }
            lastRightTrigger2 = triggerPressed2;

            // X: Return turret to home (0 ticks / 0 degrees)
            if (gamepad2.x && !lastX2) {
                turretAiming.goHome();
                telemetry.speak("Going home");
            }
            lastX2 = gamepad2.x;

            // A: Activate turretStopper servo (ball release)
            if (gamepad2.a && !lastA2 && !turretStopperActive) {
                turretStopperServo.setPosition(TURRET_STOPPER_ACTIVE);
                turretStopperStartMs = System.currentTimeMillis();
                turretStopperActive  = true;
            }
            lastA2 = gamepad2.a;

            // Auto-return stopper servo after timeout
            if (turretStopperActive &&
                    System.currentTimeMillis() - turretStopperStartMs >= TURRET_STOPPER_TIME_MS) {
                turretStopperServo.setPosition(TURRET_STOPPER_HOME);
                turretStopperActive = false;
            }

            // ═══════════════════════════════════════════
            //  SHOOTER — Auto RPM & Hood from lookup table
            // ═══════════════════════════════════════════
            if (shooterOn) {
                double targetRPM   = ShooterConstants.targetRPM(distance);
                double hoodPos     = ShooterConstants.hoodPosition(distance);
                shooterMotor.setVelocity(targetRPM);
                shooterMotor2.setVelocity(targetRPM);
                hoodServo.setPosition(hoodPos);
            } else {
                shooterMotor.setVelocity(0);
                shooterMotor2.setVelocity(0);
            }

            // ═══════════════════════════════════════════
            //  TURRET UPDATE (odometry + optional limelight)
            // ═══════════════════════════════════════════
            turretAiming.update(robotPose, robotHeading);

            // ═══════════════════════════════════════════
            //  GAMEPAD 1 — DRIVER CONTROLS
            // ═══════════════════════════════════════════

            // Speed presets (keep bumper press for full turbo in drive)
            if (gamepad1.a) maxSpeed = 0.6;
            if (gamepad1.b) maxSpeed = 0.93;

            // Intake
            if (gamepad1.right_trigger > 0.2) {
                intakeMotor.setVelocity(INTAKE_VELOCITY);   // IN
            } else if (gamepad1.right_bumper) {
                intakeMotor.setVelocity(-INTAKE_VELOCITY);  // OUT
            } else {
                intakeMotor.setVelocity(0);
            }

            // Drive
            handleDrive();

            // ═══════════════════════════════════════════
            //  TELEMETRY
            // ═══════════════════════════════════════════
            displayTelemetry(robotPose, distance, odomDist, llDist);
        }
    }

    // ─────────────────────────────────────────────────
    //  DRIVE
    // ─────────────────────────────────────────────────
    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double right   =  gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        // Left bumper = full turbo override
        double speed = gamepad1.left_bumper ? 1.0 : maxSpeed;

        frontLeftDrive.setPower(fl  * speed);
        frontRightDrive.setPower(fr * speed);
        backLeftDrive.setPower(bl   * speed);
        backRightDrive.setPower(br  * speed);
    }

    // ─────────────────────────────────────────────────
    //  HARDWARE INIT
    // ─────────────────────────────────────────────────
    private void initHardware() {
        // Drive motors
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

        imu = hardwareMap.get(IMU.class, "imu");

        // Shooter & intake
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter_motor2");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor  = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos
        turretStopperServo = hardwareMap.get(Servo.class, "turretStopper");
        hoodServo          = hardwareMap.get(Servo.class, "turret_servo");
        turretStopperServo.setDirection(Servo.Direction.REVERSE);

        // Limelight (optional)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(8);
            limelight.start();
            telemetry.addLine("Limelight: OK");
        } catch (Exception e) {
            limelight = null;
            telemetry.addLine("WARNING: Limelight not found — odometry only");
        }

        // Pedro Pathing follower (Pinpoint odometry)
        follower = teleConstants.createFollower(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(72, 72, 0));  // ← adjust to your starting position

        telemetry.update();
    }

    // ─────────────────────────────────────────────────
    //  TELEMETRY
    // ─────────────────────────────────────────────────
    private void displayTelemetry(Pose robotPose, double usedDist,
                                  double odomDist, double llDist) {
        Pose goal = turretAiming.getCurrentGoal();

        telemetry.addLine("══════════ CHAMPIONSHIP TELEOP ══════════");

        // Alliance & goal
        telemetry.addData("Alliance",
                turretAiming.getCurrentGoal().equals(goal) ? "SET" : "?");
        telemetry.addData("Goal", "X:%.1f  Y:%.1f", goal.getX(), goal.getY());
        telemetry.addLine();

        // Distance
        telemetry.addLine("─── DISTANCE ───");
        telemetry.addData("Odom Distance",  "%.1f\"", odomDist);
        telemetry.addData("LL Distance",    llDist > 0 ? String.format("%.1f\"", llDist) : "N/A");
        telemetry.addData("USED Distance",  "%.1f\"", usedDist);
        telemetry.addData("Localizer Mode", useLimelight ? "LIMELIGHT" : "ODOMETRY");
        telemetry.addLine();

        // Shooter
        telemetry.addLine("─── SHOOTER ───");
        telemetry.addData("Shooter",       shooterOn ? "ON" : "OFF");
        telemetry.addData("Target RPM",    "%.0f", ShooterConstants.targetRPM(usedDist));
        telemetry.addData("Actual RPM 1",  "%.0f", shooterMotor.getVelocity());
        telemetry.addData("Actual RPM 2",  "%.0f", shooterMotor2.getVelocity());
        telemetry.addData("Hood Position", "%.4f", ShooterConstants.hoodPosition(usedDist));
        telemetry.addLine();

        // Turret
        telemetry.addLine("─── TURRET ───");
        telemetry.addData("Aiming",       turretAiming.isAiming() ? "YES" : "NO");
        telemetry.addData("Status",       turretAiming.getStatusString());
        telemetry.addData("Control",      turretAiming.getControlMethodString());
        telemetry.addData("At Target",    turretAiming.isAtTarget() ? "✓" : "—");
        telemetry.addData("Angle Safe",   turretAiming.isTargetAngleSafe() ? "✓" : "⚠ UNSAFE");
        telemetry.addData("Curr Ticks",   turretAiming.getCurrentTicks());
        telemetry.addData("Target Ticks", turretAiming.getTargetTicks());
        telemetry.addData("Error Ticks",  turretAiming.getError());
        telemetry.addData("Curr Angle",   "%.1f°", Math.toDegrees(turretAiming.getCurrentAngle()));
        telemetry.addData("Target Angle", "%.1f°", Math.toDegrees(turretAiming.getTargetAngle()));
        telemetry.addLine();

        // Robot pose
        telemetry.addLine("─── ROBOT POSE ───");
        telemetry.addData("X",       "%.1f\"", robotPose.getX());
        telemetry.addData("Y",       "%.1f\"", robotPose.getY());
        telemetry.addData("Heading", "%.1f°",  Math.toDegrees(robotPose.getHeading()));

        telemetry.update();
    }
}