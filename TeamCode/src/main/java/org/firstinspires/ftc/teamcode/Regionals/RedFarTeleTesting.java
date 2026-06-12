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


@TeleOp(name = "RED FAR Tele (4) - Regionals", group = "0-Primary")
public class RedFarTeleTesting extends LinearOpMode {


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


    // ==================== BUTTON DEBOUNCING ====================
    private boolean lastLeftBumper = false;
    private boolean lastYButton = false;
    private boolean lastDPadUp = false;
    private boolean lastDPadDown = false;
    private boolean lastRightTrigger = false;


    @Override
    public void runOpMode() {
        initializeHardware();
        turretAiming = new TurretAiming(turretMotor, limelight);
        turretAiming.setAlliance(TurretAiming.Alliance.RED); // Default
        turretAiming.setVelocityCompensation(true);
        turretAiming.disableLimelightCorrection(); // Prioritize Odometry


        telemetry.addLine("### STEP 2: Turret & Shooter Test ###");
        telemetry.addLine("Ready to Start!");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {
            follower.update();
            Pose robotPose = follower.getPose();


            handleDrive();
            handleIntake();
            handleAutomation(robotPose);
            displayTelemetry(robotPose);
        }


        shutdownRobot();
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
        shooterMotor1.setDirection(DcMotorEx.Direction.FORWARD); // FIX: To shoot outward
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE); // FIX: To shoot outward
        shooterMotor1.setVelocityPIDFCoefficients(shooter_kP, shooter_kI, shooter_kD, shooter_kF);
        shooterMotor2.setVelocityPIDFCoefficients(shooter_kP, shooter_kI, shooter_kD, shooter_kF);


        // Servos & Turret Motor
        hoodServo = hardwareMap.get(Servo.class, "turret_servo");
        hoodServo.setDirection(Servo.Direction.REVERSE);


        turretStopperServo = hardwareMap.get(Servo.class, "turretStopper");
        turretStopperServo.setDirection(Servo.Direction.REVERSE);


        turretMotor = hardwareMap.get (DcMotorEx.class, "turret_motor");
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


        // Main automation toggle (Left Bumper)
        if (gamepad2.left_bumper && !lastLeftBumper) {
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


        // Update aiming state
        if (isAutomationActive && !turretAiming.isAiming()) turretAiming.startAiming();
        else if (!isAutomationActive && turretAiming.isAiming()) turretAiming.stopAiming();


        if (isAutomationActive) {
            // Explicitly enable/disable limelight correction
            if (gamepad2.dpad_up && !lastDPadUp) turretAiming.enableLimelightCorrection();
            if (gamepad2.dpad_down && !lastDPadDown) turretAiming.disableLimelightCorrection();


            if (gamepad2.y && !lastYButton) turretAiming.toggleVelocityCompensation();
            if (gamepad2.x) turretAiming.goHome();
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

