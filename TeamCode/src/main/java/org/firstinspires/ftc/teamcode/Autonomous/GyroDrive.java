package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class GyroDrive extends LinearOpMode {

    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    public IMU imu;

    public double distancePerRotation = (9.6 * Math.PI) / 537.7;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        frontLeft = hardwareMap.get(DcMotor.class, "M1");
        backLeft = hardwareMap.get(DcMotor.class, "M2");
        frontRight = hardwareMap.get(DcMotor.class, "M3");
        backRight = hardwareMap.get(DcMotor.class, "M4");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // IMU initialization
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            imu.resetYaw();
            imuDrive(.5,55.88,0.0);
            imuDrive(.5,-45.0,0.0);
            break;
        }
    }

    // Drive forward function with PID control
    public void imuDrive(double power, double distance, double targetHeading) {
        int targetTicks = (int) (distance / distancePerRotation);

        int frontLeftTarget = frontLeft.getCurrentPosition() + targetTicks;
        int backLeftTarget = backLeft.getCurrentPosition() + targetTicks;
        int frontRightTarget = frontRight.getCurrentPosition() + targetTicks;
        int backRightTarget = backRight.getCurrentPosition() + targetTicks;

        frontLeft.setTargetPosition(frontLeftTarget);
        backLeft.setTargetPosition(backLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backRight.setTargetPosition(backRightTarget);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double kp = 1; // Adjust proportional constant
        double ki = 0; // Adjust integral constant
        double kd = 0; // Adjust derivative constant

        double integral = 0;
        double prevError = 0;

        while (
                frontLeft.isBusy() &&
                        backLeft.isBusy() &&
                        frontRight.isBusy() &&
                        backRight.isBusy() &&
                        opModeIsActive()
        ) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetHeading - currentHeading;

            integral += error;
            double derivative = error - prevError;

            double correction = kp * error + ki * integral + kd * derivative;

            // Apply correction to individual motors
            frontLeft.setPower(power - correction);
            backLeft.setPower(power - correction);
            frontRight.setPower(power + correction);
            backRight.setPower(power + correction);

            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Correction", correction);
            telemetry.update();

            prevError = error;

            sleep(20);
        }

        // Stop motors and reset encoders after reaching the target position
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
