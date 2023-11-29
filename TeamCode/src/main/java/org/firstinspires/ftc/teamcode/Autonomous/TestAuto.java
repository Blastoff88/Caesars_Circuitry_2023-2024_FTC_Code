package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TestAuto extends LinearOpMode {

    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    public IMU imu;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        frontLeft = hardwareMap.get(DcMotor.class, "M1");
        backLeft = hardwareMap.get(DcMotor.class, "M2");
        frontRight = hardwareMap.get(DcMotor.class, "M3");
        backRight = hardwareMap.get(DcMotor.class, "M4");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            move(150, .5, "N");
            turn(0);
            move(100, .5, "S");
        }
    }

    private void move(double targetDistanceCm, double targetSpeed, String direction) {
        double initialSpeed = 0.1;
        double currentDistance = 0;

        int initialFrontLeftEncoder = frontLeft.getCurrentPosition();
        int initialBackLeftEncoder = backLeft.getCurrentPosition();
        int initialFrontRightEncoder = frontRight.getCurrentPosition();
        int initialBackRightEncoder = backRight.getCurrentPosition();

        double targetDistanceTicks = (targetDistanceCm / (Math.PI * 9.6)) * 537.7;

        while (currentDistance < targetDistanceTicks) {
            int currentFrontLeftEncoder = frontLeft.getCurrentPosition();
            int currentBackLeftEncoder = backLeft.getCurrentPosition();
            int currentFrontRightEncoder = frontRight.getCurrentPosition();
            int currentBackRightEncoder = backRight.getCurrentPosition();

            currentDistance = 0.25 * (
                    currentFrontLeftEncoder - initialFrontLeftEncoder +
                            currentBackLeftEncoder - initialBackLeftEncoder +
                            currentFrontRightEncoder - initialFrontRightEncoder +
                            currentBackRightEncoder - initialBackRightEncoder
            );

            double sigmoidFactor = 1 / (1 + Math.exp(-(12 * (currentDistance / targetDistanceTicks - 0.5))));
            double currentSpeed = initialSpeed + sigmoidFactor * (targetSpeed - initialSpeed);

            setMotorPowers(currentSpeed, currentSpeed, currentSpeed, currentSpeed, direction);
        }

        stopMotors();
    }

    private void turn(double targetHeading) {
        double turnSpeed = 0.25;
        double tolerance = 1.0;

        while (opModeIsActive()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetHeading - currentHeading;

            if (Math.abs(error) <= tolerance) {
                break;
            }

            double correction = Math.signum(error) * turnSpeed;

            setMotorPowers(-correction, correction, -correction, correction, "N");
        }

        stopMotors();
    }

    private void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower, String direction) {
        switch (direction) {
            case "N":
                break;
            case "E":
                frontLeftPower *= 1;
                frontRightPower *= -1;
                backLeftPower *= 1;
                backRightPower *= -1;
                break;
            case "S":
                frontLeftPower *= -1;
                frontRightPower *= -1;
                backLeftPower *= -1;
                backRightPower *= -1;
                break;
            case "W":
                frontLeftPower *= -1;
                frontRightPower *= 1;
                backLeftPower *= -1;
                backRightPower *= 1;
                break;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private void reset(boolean resetEncoders, boolean resetIMUYaw) {
        if (resetEncoders) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (resetIMUYaw) {
            imu.resetYaw();
        }
    }
}