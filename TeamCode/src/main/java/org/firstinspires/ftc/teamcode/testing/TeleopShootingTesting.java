package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TeleOp Shooting Testing", group = "TESTING")
public class TeleopShootingTesting extends LinearOpMode {

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;

    private boolean isShooterOn = false;
    private boolean lastRightTrigger = false;

    private final double shooter_kP = 180.0;
    private final double shooter_kI = 0.0;
    private final double shooter_kD = 1.0;
    private final double shooter_kF = 19.1;
    private final double TICKS_PER_REV = 28;
    private final double TEST_RPM = 2500; // Adjust this value for testing

    @Override
    public void runOpMode() {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter_motor2");

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor1.setVelocityPIDFCoefficients(shooter_kP, shooter_kI, shooter_kD, shooter_kF);
        shooterMotor2.setVelocityPIDFCoefficients(shooter_kP, shooter_kI, shooter_kD, shooter_kF);

        telemetry.addLine("Ready for testing!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;

            if (rightTriggerPressed && !lastRightTrigger) {
                isShooterOn = !isShooterOn;
            }
            lastRightTrigger = rightTriggerPressed;

            if (isShooterOn) {
                setShooterVelocity(TEST_RPM);
            } else {
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
            }

            double currentRPM1 = shooterMotor1.getVelocity() * 60 / TICKS_PER_REV;

            telemetry.addData("Shooter Status", isShooterOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", isShooterOn ? TEST_RPM : 0);
            telemetry.addData("Actual RPM", "%.1f", currentRPM1);
            telemetry.update();
        }

        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }

    private void setShooterVelocity(double rpm) {
        double targetTicksPerSec = rpm * TICKS_PER_REV / 60.0;
        shooterMotor1.setVelocity(targetTicksPerSec);
        shooterMotor2.setVelocity(targetTicksPerSec);
    }
}
