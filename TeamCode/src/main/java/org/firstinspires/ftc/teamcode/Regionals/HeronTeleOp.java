package org.firstinspires.ftc.teamcode.Regionals;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.pedroPathing.teleConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.TurretAiming;


@TeleOp(name = "Heron TeleOP", group = "0-Primary")
public class HeronTeleOp extends LinearOpMode {


    // ==================== HARDWARE ====================
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx intakeMotor;
    private IMU imu;
    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private Servo hoodServo;
    private Servo turretStopperServo;
    private Limelight3A limelight;
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


    // ==================== MANUAL POSE RESET (GP2 X) ====================

    /**
     * TODO: Set this to the known field position the robot will be at
     *       when the driver presses X to correct odometry drift.
     *       x, y in inches, heading in degrees (converted below).
     */
    private static final Pose MANUAL_RESET_POSE = new Pose(
            7.7534,                 // <-- TODO: replace with your desired X (inches)
            9.0261,                 // <-- TODO: replace with your desired Y (inches)
            Math.toRadians(0)  // <-- TODO: replace with your desired heading (degrees)
    );

    /**
     * IDLE      — normal operation
     * RESETTING — pose was just applied, turret driving to zero
     * ZEROED    — turret confirmed at zero, ready to resume automation
     */
    private enum PoseResetState { IDLE, RESETTING, ZEROED }
    private PoseResetState poseResetState = PoseResetState.IDLE;

    // Tracks the pose that was last applied so telemetry can confirm it
    private Pose lastAppliedResetPose = null;


    // ==================== BUTTON DEBOUNCING ====================
    private boolean lastLeftBumper = false;
    private boolean lastYButton = false;
    private boolean lastDPadUp = false;
    private boolean lastDPadDown = false;
    private boolean lastRightTrigger = false;
    private boolean lastXButton = false;


    @Override
    public void runOpMode() {
        initializeHardware();
        turretAiming = new TurretAiming(turretMotor, limelight);
        turretAiming.setAlliance(TurretAiming.Alliance.RED); // Default
        turretAiming.setVelocityCompensation(true);
        turretAiming.disableLimelightCorrection(); // Prioritize Odometry


        telemetry.addLine("### Heron TeleOP ###");
        telemetry.addLine("GP2 X = Manual Pose Reset (drift fix)");
        telemetry.addLine("Ready to Start!");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {
            follower.update();
            Pose robotPose = follower.getPose();

            handleManualPoseReset(robotPose); // Must run before automation
            handleDrive();
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
                    // Apply the pose correction immediately
                    follower.setPose(MANUAL_RESET_POSE);
                    lastAppliedResetPose = MANUAL_RESET_POSE;

                    // Pause automation and send turret home
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
                // Auto-clear once automation is re-enabled via L_Bumper
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


    private void initializeHardware() {
        // Drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");


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


        // Intake motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Shooter motors
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter_motor2");
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor1.setVelocityPIDFCoefficients(shooter_kP, shooter_kI, shooter_kD, shooter_kF);
        shooterMotor2.setVelocityPIDFCoefficients(shooter_kP, shooter_kI, shooter_kD, shooter_kF);


        // Servos & Turret Motor
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");
        hoodServo.setDirection(Servo.Direction.REVERSE);


        turretStopperServo = hardwareMap.get(Servo.class, "turretStopper");
        turretStopperServo.setDirection(Servo.Direction.REVERSE);


        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretStopperServo.setPosition(TURRET_STOPPER_HOME);


        // IMU & Limelight
        imu = hardwareMap.get(IMU.class, "imu");
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) { limelight = null; }


        // Odometry
        follower = teleConstants.createFollower(hardwareMap, telemetry);
        follower.setStartingPose(new Pose(92.8018, 19.9742, Math.toRadians(0)));
    }


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


        // Main automation toggle (Left Bumper) — blocked while pose is RESETTING
        if (poseResetState != PoseResetState.RESETTING
                && gamepad2.left_bumper && !lastLeftBumper) {
            isAutomationActive = !isAutomationActive;
            if (!isAutomationActive) isShooterOn = false;
        }
        lastLeftBumper = gamepad2.left_bumper;


        // Shooter motor toggle (Right Trigger)
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;
        if (isAutomationActive && rightTriggerPressed && !lastRightTrigger) {
            isShooterOn = !isShooterOn;
        }
        lastRightTrigger = rightTriggerPressed;


        // Turret Stopper (Right Bumper)
        if (gamepad2.right_bumper && !isTurretStopperActive) {
            turretStopperServo.setPosition(TURRET_STOPPER_ACTIVE);
            turretStopperStartTime = System.currentTimeMillis();
            isTurretStopperActive = true;
        }
        if (isTurretStopperActive && System.currentTimeMillis() - turretStopperStartTime >= TURRET_STOPPER_TIME_MS) {
            turretStopperServo.setPosition(TURRET_STOPPER_HOME);
            isTurretStopperActive = false;
        }


        // Aiming state sync — don't fight goHome() while resetting
        if (poseResetState != PoseResetState.RESETTING) {
            if (isAutomationActive && !turretAiming.isAiming()) turretAiming.startAiming();
            else if (!isAutomationActive && turretAiming.isAiming()) turretAiming.stopAiming();
        }


        if (isAutomationActive) {
            if (gamepad2.dpad_up && !lastDPadUp) turretAiming.enableLimelightCorrection();
            if (gamepad2.dpad_down && !lastDPadDown) turretAiming.disableLimelightCorrection();

            if (gamepad2.y && !lastYButton) turretAiming.toggleVelocityCompensation();
            if (gamepad2.back) { isAutomationActive = false; isShooterOn = false; }
        }
        lastDPadUp = gamepad2.dpad_up;
        lastDPadDown = gamepad2.dpad_down;
        lastYButton = gamepad2.y;


        // Main Logic Update
        turretAiming.update(robotPose, robotPose.getHeading());


        if (isAutomationActive) {
            double distance = Math.hypot(turretAiming.getCurrentGoal().getX() - robotPose.getX(), turretAiming.getCurrentGoal().getY() - robotPose.getY());
            targetRPM = ShooterConstants.targetRPM(distance);
            hoodPosition = ShooterConstants.hoodPosition(distance);

            hoodServo.setPosition(hoodPosition);

            if (isShooterOn) setShooterVelocity(targetRPM);
            else { shooterMotor1.setPower(0); shooterMotor2.setPower(0); }
        } else {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
        }
    }


    private void handleDrive() {
        if (gamepad1.a) maxSpeed = 0.55;
        if (gamepad1.b) maxSpeed = 0.93;


        double forward = -gamepad1.left_stick_y * maxSpeed;
        double right = gamepad1.left_stick_x * maxSpeed;
        double rotate = gamepad1.right_stick_x * maxSpeed;


        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;


        frontLeftDrive.setPower(fl); frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl); backRightDrive.setPower(br);
    }


    private void handleIntake() {
        if (gamepad1.right_trigger > 0.1) {
            intakeMotor.setVelocity(INTAKE_VELOCITY); // Intake In
        } else if (gamepad1.right_bumper) {
            intakeMotor.setVelocity(-INTAKE_VELOCITY); // Intake Out
        } else {
            intakeMotor.setVelocity(0);
        }
    }


    private void setShooterVelocity(double rpm) {
        double targetTicksPerSec = rpm * TICKS_PER_REV / 60.0;
        shooterMotor1.setVelocity(targetTicksPerSec);
        shooterMotor2.setVelocity(targetTicksPerSec);
    }


    private void shutdownRobot() {
        turretAiming.stopAiming();
        shooterMotor1.setPower(0); shooterMotor2.setPower(0);
        intakeMotor.setPower(0);
        frontLeftDrive.setPower(0); frontRightDrive.setPower(0);
        backLeftDrive.setPower(0); backRightDrive.setPower(0);
    }


    private void displayTelemetry(Pose robotPose) {
        double distance = Math.hypot(turretAiming.getCurrentGoal().getX() - robotPose.getX(), turretAiming.getCurrentGoal().getY() - robotPose.getY());
        double currentRPM1 = shooterMotor1.getVelocity() * 60 / TICKS_PER_REV;


        telemetry.addLine("--- GP1: Drive/Intake | GP2: Operate ---");
        telemetry.addData("Automation (L_Bumper)", isAutomationActive ? "ON" : "OFF");
        telemetry.addData("Shooter (R_Trigger)", isShooterOn ? "ON" : "OFF");
        telemetry.addData("Intake Vel", "%.1f", intakeMotor.getVelocity());
        telemetry.addLine();

        // ── Manual Pose Reset Telemetry ──────────────────────────────────────
        telemetry.addLine("--- Manual Pose Reset (GP2 X) ---");
        telemetry.addData("Reset State", poseResetState.name());
        telemetry.addData("Reset Pose (target)",
                "(%.2f, %.2f, %.1f°)",
                MANUAL_RESET_POSE.getX(),
                MANUAL_RESET_POSE.getY(),
                Math.toDegrees(MANUAL_RESET_POSE.getHeading()));
        telemetry.addData("Follower Pose (live)",
                "(%.2f, %.2f, %.1f°)",
                robotPose.getX(),
                robotPose.getY(),
                Math.toDegrees(robotPose.getHeading()));
        if (lastAppliedResetPose != null) {
            telemetry.addData("Last Applied Reset",
                    "(%.2f, %.2f, %.1f°)",
                    lastAppliedResetPose.getX(),
                    lastAppliedResetPose.getY(),
                    Math.toDegrees(lastAppliedResetPose.getHeading()));
        } else {
            telemetry.addData("Last Applied Reset", "None yet");
        }
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

        telemetry.addLine("--- GP2 Aiming Controls ---");
        telemetry.addData("Alliance (A/B)", currentAlliance);
        telemetry.addData("Limelight (D-Up/D-Down)", turretAiming.isUsingLimelight() ? "ON" : "OFF");
        telemetry.addData("Vel Comp (Y)", turretAiming.isUsingVelocityCompensation() ? "ON" : "OFF");
        telemetry.addLine();


        telemetry.addLine("--- Turret & Shooter ---");
        telemetry.addData("Distance", "%.1f in", distance);
        telemetry.addData("Target RPM", "%.1f", targetRPM);
        telemetry.addData("Actual RPM", "%.1f", currentRPM1);
        telemetry.addData("Hood Target Pos", "%.3f", hoodPosition);
        telemetry.addData("Hood Actual Pos", "%.3f", hoodServo.getPosition());
        telemetry.addData("Stopper State", isTurretStopperActive ? "ACTIVE" : "HOME");
        telemetry.addData("Stopper Actual Pos", "%.3f", turretStopperServo.getPosition());
        telemetry.update();
    }
}