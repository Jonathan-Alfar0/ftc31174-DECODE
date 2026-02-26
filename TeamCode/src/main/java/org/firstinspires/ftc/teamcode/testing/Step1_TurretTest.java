package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.teleConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.TurretAiming;

/**
 * STEP 1: Test Turret Aiming with Shooting-While-Moving Compensation
 *
 * This OpMode tests turret aiming including velocity compensation.
 * No shooting, no intake - just turret rotation and aiming logic.
 *
 * CONTROLS:
 *
 * Gamepad 1:
 *   Left Stick:      Drive (strafe + forward)
 *   Right Stick X:   Rotate
 *   A:               Slow mode (0.6 speed)
 *   B:               Fast mode (0.77 speed)
 *
 * Gamepad 2:
 *   A:               Set BLUE alliance
 *   B:               Set RED alliance
 *   LEFT BUMPER:     Toggle turret aiming on/off
 *   RIGHT BUMPER:    Toggle Limelight correction
 *   X:               Go to home position
 *   Y:               Toggle velocity compensation (shooting while moving)
 *   BACK:            Stop aiming
 *
 * SETUP CHECKLIST:
 * 1. Set goal positions in TurretAiming.java (RED_GOAL, BLUE_GOAL)
 * 2. Set turret angle limits (TURRET_MIN_ANGLE_RAD, TURRET_MAX_ANGLE_RAD)
 * 3. Calibrate TURRET_TICKS_PER_RADIAN
 * 4. Tune TIME_OF_FLIGHT_SECS in TurretAiming.java for shooting while moving
 * 5. Test!
 *
 * TUNING SHOOTING-WHILE-MOVING:
 * - Drive straight toward goal at consistent speed, aim, and observe
 * - Shots missing BEHIND goal → increase TIME_OF_FLIGHT_SECS
 * - Shots missing AHEAD of goal → decrease TIME_OF_FLIGHT_SECS
 * - Use Y button to toggle compensation on/off to compare
 */
@TeleOp(name = "STEP 1 - Turret Aiming Test", group = "TESTING")
public class Step1_TurretTest extends LinearOpMode {

    // ==================== HARDWARE ====================
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private Follower follower;  // For Pinpoint odometry

    // ==================== TURRET AIMING ====================
    private TurretAiming turretAiming;

    // ==================== DRIVE STATE ====================
    private double maxSpeed = 0.77;
    private boolean fieldOriented = false;

    // Button debouncing
    private boolean lastLeftBumper   = false;
    private boolean lastRightBumper  = false;
    private boolean lastYButton      = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Create turret aiming controller
        turretAiming = new TurretAiming(turretMotor, limelight);

        // Set default alliance
        turretAiming.setAlliance(TurretAiming.Alliance.BLUE);

        // Velocity compensation on by default
        turretAiming.setVelocityCompensation(true);

        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("  STEP 1: TURRET AIMING TEST");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine();
        telemetry.addLine("Tests turret aiming + shoot-while-moving");
        telemetry.addLine("No shooting or intake yet!");
        telemetry.addLine();
        telemetry.addLine("L BUMPER : Toggle aim");
        telemetry.addLine("R BUMPER : Toggle Limelight");
        telemetry.addLine("Y        : Toggle vel compensation");
        telemetry.addLine("X        : Go home");
        telemetry.addLine("A / B    : Set alliance");
        telemetry.addLine();
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== UPDATE ODOMETRY =====
            follower.update();
            Pose robotPose   = follower.getPose();
            double robotHeading = robotPose.getHeading();

            // ===== ALLIANCE SELECTION =====
            if (gamepad2.a) {
                turretAiming.setAlliance(TurretAiming.Alliance.BLUE);
                telemetry.speak("Blue alliance");
            }
            if (gamepad2.b) {
                turretAiming.setAlliance(TurretAiming.Alliance.RED);
                telemetry.speak("Red alliance");
            }

            // ===== TURRET AIMING CONTROLS =====

            // LEFT BUMPER: Toggle aiming on/off (edge detect)
            if (gamepad2.left_bumper && !lastLeftBumper) {
                turretAiming.toggleAiming();
                telemetry.speak(turretAiming.isAiming() ? "Aiming" : "Stopped");
            }
            lastLeftBumper = gamepad2.left_bumper;

            // RIGHT BUMPER: Toggle Limelight correction (edge detect)
            if (gamepad2.right_bumper && !lastRightBumper) {
                turretAiming.toggleLimelightCorrection();
                telemetry.speak(turretAiming.isUsingLimelight() ? "Limelight on" : "Limelight off");
            }
            lastRightBumper = gamepad2.right_bumper;

            // Y: Toggle velocity compensation (edge detect)
            if (gamepad2.y && !lastYButton) {
                turretAiming.toggleVelocityCompensation();
                telemetry.speak(turretAiming.isUsingVelocityCompensation()
                        ? "Vel comp on" : "Vel comp off");
            }
            lastYButton = gamepad2.y;

            // X: Go home
            if (gamepad2.x) {
                turretAiming.goHome();
                telemetry.speak("Going home");
            }

            // BACK: Stop aiming
            if (gamepad2.back) {
                turretAiming.stopAiming();
            }

            // ===== UPDATE TURRET =====
            turretAiming.update(robotPose, robotHeading);

            // ===== DRIVE =====
            handleDrive();

            // ===== TELEMETRY =====
            displayTelemetry(robotPose);
        }
    }

    // ==================== HARDWARE INITIALIZATION ====================

    private void initializeHardware() {
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

        // IMU for field-oriented drive
        imu = hardwareMap.get(IMU.class, "imu");

        // Turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Limelight (optional - safely caught if not present)
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(8);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
            telemetry.addLine("WARNING: Limelight not found - odometry only");
        }

        // Pinpoint odometry through Pedro Pathing
        follower = teleConstants.createFollower(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(72, 72, 0));  // TODO: set your actual starting pose
    }

    // ==================== DRIVE CONTROL ====================

    private void handleDrive() {
        // Speed control
        if (gamepad1.a) maxSpeed = 0.6;
        if (gamepad1.b) maxSpeed = 0.77;

        double forward = -gamepad1.left_stick_y;
        double right   =  gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;

        // Field-oriented drive (optional — toggle fieldOriented flag if you add a button)
        if (fieldOriented) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw();
            double temp =  forward * Math.cos(heading) + right * Math.sin(heading);
            right       = -forward * Math.sin(heading) + right * Math.cos(heading);
            forward     = temp;
        }

        // Mecanum drive mixing
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        // Normalize so no value exceeds 1.0
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl  * maxSpeed);
        frontRightDrive.setPower(fr * maxSpeed);
        backLeftDrive.setPower(bl   * maxSpeed);
        backRightDrive.setPower(br  * maxSpeed);
    }

    // ==================== TELEMETRY ====================

    private void displayTelemetry(Pose robotPose) {
        Pose goal = turretAiming.getCurrentGoal();

        double dx       = goal.getX() - robotPose.getX();
        double dy       = goal.getY() - robotPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        double velX     = turretAiming.getRobotVelocityX();
        double velY     = turretAiming.getRobotVelocityY();
        double speed    = Math.sqrt(velX * velX + velY * velY);

        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("  TURRET AIMING TEST");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine();

        // ── Alliance & Goal ──
        telemetry.addLine("─── ALLIANCE & GOAL ───");
        telemetry.addData("Alliance",     turretAiming.getCurrentGoal());
        telemetry.addData("Goal",         "X:%.1f  Y:%.1f", goal.getX(), goal.getY());
        telemetry.addData("Distance",     "%.1f\"", distance);
        telemetry.addLine();

        // ── Turret Status ──
        telemetry.addLine("─── TURRET STATUS ───");
        telemetry.addData("Aiming",       turretAiming.isAiming()        ? "YES"    : "NO");
        telemetry.addData("Status",       turretAiming.getStatusString());
        telemetry.addData("Control",      turretAiming.getControlMethodString());
        telemetry.addData("At Target",    turretAiming.isAtTarget()       ? "✓"     : "X");
        telemetry.addData("Angle Safe",   turretAiming.isTargetAngleSafe() ? "✓"   : "⚠ UNSAFE");
        telemetry.addLine();

        // ── Turret Position ──
        telemetry.addLine("─── TURRET POSITION ───");
        telemetry.addData("Current Ticks",  turretAiming.getCurrentTicks());
        telemetry.addData("Target Ticks",   turretAiming.getTargetTicks());
        telemetry.addData("Error (ticks)",  turretAiming.getError());
        telemetry.addData("Target Angle",   "%.1f°", Math.toDegrees(turretAiming.getTargetAngle()));
        telemetry.addData("Current Angle",  "%.1f°", Math.toDegrees(turretAiming.getCurrentAngle()));
        telemetry.addLine();

        // ── Shooting While Moving ──
        telemetry.addLine("─── SHOOTING WHILE MOVING ───");
        telemetry.addData("Vel Comp",        turretAiming.isUsingVelocityCompensation() ? "ON" : "OFF");
        telemetry.addData("Robot Vel X",     "%.1f in/s", velX);
        telemetry.addData("Robot Vel Y",     "%.1f in/s", velY);
        telemetry.addData("Robot Speed",     "%.1f in/s", speed);
        telemetry.addData("Time of Flight",  "%.3f sec",  turretAiming.getLastTimeOfFlight());
        telemetry.addData("Lookahead Target","X:%.1f  Y:%.1f",
                turretAiming.getLookaheadTargetX(),
                turretAiming.getLookaheadTargetY());
        telemetry.addLine();

        // ── Robot Position ──
        telemetry.addLine("─── ROBOT POSITION ───");
        telemetry.addData("X",       "%.1f\"", robotPose.getX());
        telemetry.addData("Y",       "%.1f\"", robotPose.getY());
        telemetry.addData("Heading", "%.1f°",  Math.toDegrees(robotPose.getHeading()));
        telemetry.addLine();

        // ── Controls Reminder ──
        telemetry.addLine("─── CONTROLS ───");
        telemetry.addLine("L Bumper : Toggle Aim");
        telemetry.addLine("R Bumper : Toggle Limelight");
        telemetry.addLine("Y        : Toggle Vel Comp");
        telemetry.addLine("X        : Go Home");
        telemetry.addLine("A / B    : Set Alliance");
        telemetry.addLine("G1 A/B   : Slow / Fast");

        telemetry.update();
    }
}