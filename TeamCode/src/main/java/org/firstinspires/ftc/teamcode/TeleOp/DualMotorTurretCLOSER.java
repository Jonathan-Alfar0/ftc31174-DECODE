package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Dual Motor Turret CLOSER", group = "TeleOp")
public class DualMotorTurretCLOSER extends LinearOpMode {

    DcMotorEx turretMotor1;
    DcMotorEx turretMotor2;

    // Shooter target
    final double MAX_RPM = 6000;           // motor max RPM
    double targetVelocity = 0.25 * MAX_RPM; // match old 0.7 power

    // PIDF coefficients
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double kF = 0.0;

    // Last button states
    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;
    boolean lastY = false;

    @Override
    public void runOpMode() {

        // Initialize shooter motors
        turretMotor1 = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        turretMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor1.setDirection(DcMotorEx.Direction.REVERSE);

        turretMotor2 = hardwareMap.get(DcMotorEx.class, "shooter_motor2"); // NOTE: Assumes "shooter_motor_2" is the name in your robot configuration
        turretMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor2.setDirection(DcMotorEx.Direction.FORWARD); // NOTE: Assumes the second motor is also reversed

        // Set initial PIDF coefficients
        turretMotor1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        turretMotor2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        waitForStart();

        while (opModeIsActive()) {

            // === Adjust Target RPM with Triggers ===
            double triggerFactor = 50.0; // Controls how fast the RPM changes
            targetVelocity += gamepad1.right_trigger * triggerFactor;
            targetVelocity -= gamepad1.left_trigger * triggerFactor;

            // Clamp the target velocity to be within the motor's limits
            if (targetVelocity > MAX_RPM) {
                targetVelocity = MAX_RPM;
            }
            if (targetVelocity < 0) {
                targetVelocity = 0;
            }

            // === Maintain shooter velocity ===
            final double TICKS_PER_REV = 28; // adjust for your motor
            double targetTicksPerSec = targetVelocity * TICKS_PER_REV / 60.0;
            turretMotor1.setVelocity(-targetTicksPerSec);
            turretMotor2.setVelocity(-targetTicksPerSec);

            // === PIDF live tuning buttons ===
            if (gamepad1.a && !lastA) {
                kP += 50;
                turretMotor1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
                turretMotor2.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }
            if (gamepad1.b && !lastB) {
                kP -= 15.0;
                if (kP < 0) kP = 0;
                turretMotor1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
                turretMotor2.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }
            if (gamepad1.x && !lastX) {
                kF += 50.0;
                turretMotor1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
                turretMotor2.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }
            if (gamepad1.y && !lastY) {
                kF -= 15.0;
                if (kF < 0) kF = 0;
                turretMotor1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
                turretMotor2.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }

            // === Update last button states ===
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // === Telemetry ===
            double currentRPM1 = turretMotor1.getVelocity() * 60 / TICKS_PER_REV;
            double currentRPM2 = turretMotor2.getVelocity() * 60 / TICKS_PER_REV;
            telemetry.addData("Target RPM", "%.1f", targetVelocity);
            telemetry.addData("Current RPM 1", "%.1f", currentRPM1);
            telemetry.addData("Current RPM 2", "%.1f", currentRPM2);
            telemetry.addData("kP", "%.2f", kP);
            telemetry.addData("kF", "%.2f", kF);
            telemetry.update();
        }
    }
}
