package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class Zacs_Test_Autonomous extends LinearOpMode {

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
            imu.resetYaw();
            move(83.82, .5);
            move(83,.5);

        }
    }

    public void move(double distance, double speed) {
        reset(true,false);
        double totalDistance = distance * ((9.6*Math.PI) / 537.7);
        double sectionLength = totalDistance/3;

        double accelerationFactor = speed-0.05/sectionLength;
        double decelerationFactor = -speed/sectionLength;

        double currentPosition = 0;
        while (currentPosition < totalDistance) {
            frontLeft.setPower(accelerationFactor*frontLeft.getCurrentPosition()+.1);
            frontRight.setPower(accelerationFactor*frontRight.getCurrentPosition()+.1);
            backLeft.setPower(accelerationFactor*backLeft.getCurrentPosition()+.1);
            backRight.setPower(accelerationFactor*backRight.getCurrentPosition()+.1);
        }
        while (currentPosition < totalDistance) {
            frontLeft.setPower(decelerationFactor-.1);
            frontRight.setPower(decelerationFactor-.1);
            backLeft.setPower(decelerationFactor-.1);
            backRight.setPower(decelerationFactor-.1);
        }
    }
    public void reset(boolean encoderReset, boolean imuReset) {
        if(encoderReset) {
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(imuReset) {
            imu.resetYaw();
        }
    }

}
