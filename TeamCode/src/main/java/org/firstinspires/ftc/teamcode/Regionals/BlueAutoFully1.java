package org.firstinspires.ftc.teamcode.Regionals;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;

import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shooterConstants.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooterConstants.TurretAiming;

@Autonomous(name = "FULL Auto for Blue 1 - TESTING", group = "Autonomous")
@Configurable
public class BlueAutoFully1 extends OpMode {

    public Follower follower;

    private DcMotorEx intakeMotor;
    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private Servo hoodServo;
    private Servo turretStopperServo;

    private TurretAiming turretAiming;

    private final double TICKS_PER_REV = 28;
    private static final double INTAKE_VELOCITY = 1800;
    private static final double TURRET_STOPPER_HOME = 0.45;
    private static final double TURRET_STOPPER_ACTIVE = 0.2;
    private static final long TURRET_STOPPER_TIME_MS = 1500;

    private final double shooter_kP = 180.00;
    private final double shooter_kI = 0.0;
    private final double shooter_kD = 1.0;
    private final double shooter_kF = 19.1;

    // ── PATH STATE MACHINE ──
    private int pathState = 0;
    private int lastState = -1;
    private ElapsedTime stateTimer = new ElapsedTime();

    // ── SHOOT STATE MACHINE ──
    private int shootStep = 0;
    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean isTurretStopperActive = false;
    private long turretStopperStartTime = 0;

    private Paths paths;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(34.206, 136.7615, Math.toRadians(270)));

        paths = new Paths(follower);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        turretStopperServo.setPosition(TURRET_STOPPER_HOME);

        turretAiming = new TurretAiming(turretMotor, null);
        turretAiming.setAlliance(TurretAiming.Alliance.BLUE);
        turretAiming.setVelocityCompensation(true);
        turretAiming.disableLimelightCorrection();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        turretAiming.update(follower.getPose(), follower.getPose().getHeading());

        // Non-blocking turret stopper auto-return (same pattern as teleop)
        if (isTurretStopperActive &&
                System.currentTimeMillis() - turretStopperStartTime >= TURRET_STOPPER_TIME_MS) {
            turretStopperServo.setPosition(TURRET_STOPPER_HOME);
            isTurretStopperActive = false;
        }

        pathState = autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shoot Step", shootStep);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // =====================================================================
    //  SHOOT SEQUENCE — non-blocking, mirrors teleop exactly:
    //    0. Start aiming + spin up shooter + set hood
    //    1. Wait 1.0s for flywheel to reach speed (continuously update)
    //    2. Activate turret stopper (fire ring)
    //    3. Wait for stopper cycle to complete
    //    4. Stop shooter, stop aiming → return true (done)
    // =====================================================================
    private boolean shootSequence() {
        Pose robotPose = follower.getPose();
        double distance = Math.hypot(
                turretAiming.getCurrentGoal().getX() - robotPose.getX(),
                turretAiming.getCurrentGoal().getY() - robotPose.getY());
        double targetRPM     = ShooterConstants.targetRPM(distance);
        double targetHoodPos = ShooterConstants.hoodPosition(distance);

        // Always keep hood and shooter updated
        hoodServo.setPosition(targetHoodPos);
        setShooterVelocity(targetRPM);

        switch (shootStep) {

            case 0: // Run intake to feed ring to stopper
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                shootTimer.reset();
                shootStep = 1;
                return false;

            case 1: // Brief pause for ring to reach stopper
                if (shootTimer.seconds() >= 0.1) {
                    shootStep = 2;
                }
                return false;

            case 2: // Fire stopper
                turretStopperServo.setPosition(TURRET_STOPPER_ACTIVE);
                turretStopperStartTime = System.currentTimeMillis();
                isTurretStopperActive  = true;
                shootStep = 3;
                return false;

            case 3: // Wait for stopper to return home
                if (!isTurretStopperActive) {
                    shootStep = 4;
                }
                return false;

            case 4: // Stop intake, done
                intakeMotor.setVelocity(0);
                shootStep = 0;
                return true;

            default:
                shootStep = 0;
                return true;
        }
    }

    /* ================= HARDWARE HELPERS ================= */

    private void setShooterVelocity(double rpm) {
        double targetTicksPerSec = rpm * TICKS_PER_REV / 60.0;
        shooterMotor1.setVelocity(targetTicksPerSec);
        shooterMotor2.setVelocity(targetTicksPerSec);
    }

    /* ================= PATHS ================= */

    public static class Paths {
        public PathChain preLoad;
        public PathChain pickup1;
        public PathChain shoot1;
        public PathChain openGate1;
        public PathChain intake2;
        public PathChain shoot2;
        public PathChain intake3;
        public PathChain shoot3;
        public PathChain intake4;
        public PathChain shoot4;

        public Paths(Follower follower) {

            preLoad = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(34.206, 136.7615),
                                    new Pose(60.9645, 82.8172)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();

            pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.9645, 82.8172),
                                    new Pose(51.704, 56.003),
                                    new Pose(21.1202, 59.2345)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.1202, 59.2345),
                                    new Pose(60.9645, 82.8172)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            openGate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.9645, 82.8172),
                                    new Pose(43.024, 64.217),
                                    new Pose(17.0886, 61.5776)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();

            intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.0886, 61.5776),
                                    new Pose(15.9939, 52.771)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(154))
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.9939, 52.771),
                                    new Pose(60.9645, 82.8172)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(154), Math.toRadians(180))
                    .build();

            intake3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.9645, 82.8172),
                                    new Pose(60.560, 29.275),
                                    new Pose(16.413, 36.092)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.413, 36.092),
                                    new Pose(60.9645, 82.8172)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intake4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.9645, 82.8172),
                                    new Pose(24.0513, 82.8172)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.0513, 82.8172),
                                    new Pose(63.9281, 109.8185)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    /* ================= STATE MACHINE ================= */

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0: // drive preload to shoot position — start aiming + shooter while moving
                turretAiming.startAiming();
                setShooterVelocity(2530);
                hoodServo.setPosition(0.173);
                follow(paths.preLoad);
                if (!follower.isBusy()) advance();
                break;

            case 1: // shoot preload
                if (shootSequence())
                    advance();
                break;

            case 2: // intake ring 1
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.pickup1);
                if (!follower.isBusy())
                    advance();
                break;

            case 3: // drive back to shoot position
                intakeMotor.setVelocity(0);
                follow(paths.shoot1);
                if (!follower.isBusy())
                    advance();
                break;

            case 4: // shoot ring 1
                if (shootSequence())
                    advance();
                break;

            case 5: // open gate
                follow(paths.openGate1);
                if (!follower.isBusy()) advance();
                break;

            case 6: // wait at gate with intake running
                if (stateTimer.seconds() >= 0.01)
                    advance(); // tune this
                break;

            case 7: // intake ring 2
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.intake2);
                if (!follower.isBusy()) advance();
                break;

            case 8: // wait at intake2 position with intake still running
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                if (stateTimer.seconds() >= 0.52) advance(); // tune this
                break;

            case 9: // drive back to shoot position
                intakeMotor.setVelocity(0);
                follow(paths.shoot2);
                if (!follower.isBusy()) advance();
                break;

            case 10: // shoot ring 2
                if (shootSequence()) advance();
                break;

            case 11: // intake ring 3
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.intake3);
                if (!follower.isBusy()) advance();
                break;

            case 12: // drive back to shoot position
                intakeMotor.setVelocity(0);
                follow(paths.shoot3);
                if (!follower.isBusy()) advance();
                break;

            case 13: // shoot ring 3
                if (shootSequence()) advance();
                break;

            case 14: // intake ring 3
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.intake4);
                if (!follower.isBusy()) advance();
                break;

            case 15: // drive back to shoot position
                intakeMotor.setVelocity(0);
                follow(paths.shoot4);
                if (!follower.isBusy()) advance();
                break;

            case 16: // shoot ring 3
                if (shootSequence()) advance();
                break;

            default: // DONE — reset turret to zero
                turretAiming.goHome();
                intakeMotor.setVelocity(0);
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
                break;
        }
        return pathState;
    }

    /* ================= HELPERS ================= */

    private void follow(PathChain path) {
        follow(path, false);
    }

    private void follow(PathChain path, boolean holdPoint) {
        if (pathState != lastState) {
            follower.followPath(path, holdPoint);
            lastState = pathState;
        }
    }

    private void advance() {
        pathState++;
        stateTimer.reset();
        shootStep = 0;
    }
}