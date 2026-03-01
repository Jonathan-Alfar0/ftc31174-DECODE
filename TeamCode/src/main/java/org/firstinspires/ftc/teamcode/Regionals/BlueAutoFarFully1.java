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

@Autonomous(name = "AUTO BLUE FAR (2) - Regionals", group = "0_Primary")
@Configurable
public class BlueAutoFarFully1 extends OpMode {

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
        follower.setStartingPose(new Pose(59.6778, 9.9967, Math.toRadians(180)));

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
        public PathChain getOut;

        public Paths(Follower follower) {

            preLoad = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.6778, 9.9967),
                                    new Pose(61.4247, 19.9742)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.4247, 19.9742),
                                    new Pose(56.98849797023004, 59.578484438430316),
                                    new Pose(22.5683, 61.3105)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
// Actually opening gate
            shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(22.5683, 61.3105),
                                    new Pose(30.545331529093364, 68.03247631935047),
                                    new Pose(21.1093, 65.3213)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
// Actually first shot
            openGate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(21.1093, 65.3213),
                                    new Pose(52.63531799729364, 53.598782138024355),
                                    new Pose(61.4247, 19.9742)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.4247, 19.9742),
                                    new Pose(51.000676589986455, 36.654939106901224),
                                    new Pose(27.229, 36.748)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(27.229, 36.748),
                                    new Pose(61.4247, 19.9742)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intake3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.4247, 19.9742),
                                    new Pose(55.6190798376184, 8.815967523680643),
                                    new Pose(19.7226, 11.0044)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.7226, 11.0044),
                                    new Pose(61.4247, 19.9742)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intake4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.4247, 19.9742),
                                    new Pose(19.9681, 27.2707)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.9681, 27.2707),
                                    new Pose(61.4247, 19.9742)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            getOut = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.4247, 19.9742),
                                    new Pose(51.1982, 19.9742)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    /* ================= STATE MACHINE ================= */

    public int autonomousPathUpdate() {
        switch (pathState) {

            // ── PRELOAD ──────────────────────────────────────────────────
            case 0:
                // Start aiming + spin shooter while driving to shoot spot.
                // Flywheel has the whole path length to reach speed.
                turretAiming.startAiming();
                setShooterVelocity(3100);
                hoodServo.setPosition(0.173);
                follow(paths.preLoad);
                if (!follower.isBusy()) advance();
                break;

            case 1:
                // Wait for flywheel to reach speed before shooting preload.
                // Turret is still aiming and shooter is still spinning during this wait.
                // TODO: Tune this — 0.75s is a reasonable starting point.
                turretAiming.startAiming();
                setShooterVelocity(3100);
                hoodServo.setPosition(0.173);
                if (stateTimer.seconds() >= 0.65) advance();
                break;

            case 2:
                // Shoot preload ring
                if (shootSequence())
                    advance();
                break;

            // ── RING 1 — intake → gate → return & shoot ─────────────────
            case 3:
                // Drive to ring 1 with intake running
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.pickup1);
                if (!follower.isBusy())
                    advance();
                break;

            case 4:
                // Stop intake, drive to gate position to open it (shoot1 path).
                // No shot here — just opening the gate.
                intakeMotor.setVelocity(0);
                follow(paths.shoot1);
                if (!follower.isBusy())
                    advance();
                break;

            case 5:
                // Drive back from gate to shoot position (openGate1 path).
                // Turret is live the whole way, shoot ring 1 once we arrive.
                follow(paths.openGate1);
                if (!follower.isBusy()) advance();
                break;

            case 6:
                // Shoot ring 1 at the shoot position
                if (shootSequence())
                    advance();
                break;

            // ── RING 2 — intake → shoot ──────────────────────────────────
            case 7:
                // Drive to ring 2 with intake running
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.intake2);
                if (!follower.isBusy()) advance();
                break;

            case 8:
                // Drive back to shoot position
                intakeMotor.setVelocity(0);
                follow(paths.shoot2);
                if (!follower.isBusy())
                    advance();
                break;

            case 9:
                // Shoot ring 2
                if (shootSequence())
                    advance();
                break;

            // ── RING 3 — intake → shoot ──────────────────────────────────
            case 10:
                // Drive to ring 3 with intake running
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.intake3);
                if (!follower.isBusy()) advance();
                break;

            case 11:
                // Drive back to shoot position
                intakeMotor.setVelocity(0);
                follow(paths.shoot3);
                if (!follower.isBusy())
                    advance();
                break;

            case 12:
                // Shoot ring 3
                if (shootSequence())
                    advance();
                break;

            // ── RING 4 — intake → shoot ──────────────────────────────────
            case 13:
                // Drive to ring 4 with intake running
                intakeMotor.setVelocity(-INTAKE_VELOCITY);
                follow(paths.intake4);
                if (!follower.isBusy())
                    advance();
                break;

            case 14:
                // Drive back to shoot position
                intakeMotor.setVelocity(0);
                follow(paths.shoot4);
                if (!follower.isBusy())
                    advance();
                break;

            case 15:
                // Shoot ring 4
                if (shootSequence())
                    advance();
                break;

            // ── GET OUT ──────────────────────────────────────────────────
            case 16:
                // Zero turret while driving to park position.
                // Everything else shuts down once path completes.
                follow(paths.getOut);
                if (!follower.isBusy()) advance();
                break;

            default:
                // Fully done — everything off
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