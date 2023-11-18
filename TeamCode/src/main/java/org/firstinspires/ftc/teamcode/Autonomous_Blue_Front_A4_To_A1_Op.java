/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Autonomous_Blue_Front_A4_To_A1_Op (Blocks to Java)")
public class Autonomous_Blue_Front_A4_To_A1_Op extends LinearOpMode {

    private IMU imu;
    private DcMotor left_back_drive;
    private DcMotor left_front_drive;
    private DcMotor right_back_drive;
    private DcMotor right_front_drive;
    private Servo SBasket;

    double heading_tolerance;
    int headingError;
    double currentYawAngle;
    double DriveSpeed;
    double turnSpeed;
    int motorCurrentPositionAverage;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    /*@Override
    public void runOpMode() {
        double targetYawRangeFinder;
        double x;

        imu = hardwareMap.get(IMU.class, "imu");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        SBasket = hardwareMap.get(Servo.class, "S-Basket");

        InitializeMotors();
        InitializeIMU();
        HeadingErrorInitialization();
        DriveSpeed = 0.5;
        waitForStart();
        if (opModeIsActive()) {
            DriveForward_Back(2.25);
            Turn(0);
            DriveForward_Back(-2.1);
            Turn(91);
            DriveForward_Back(8);
            while (opModeIsActive()) {
                telemetry.addData("Yaw", currentYawAngle);
                telemetry.addData("turnSpeed", turnSpeed);
                telemetry.addData("targetYawRangeFinder", targetYawRangeFinder);
                telemetry.addData("targetAngleYaw", x);
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
   /* private void Turn(int targetAngleYaw) {
        int turnLoopCheckNumber;

        turnLoopCheckNumber = 0;
        while (1 == 1) {
            currentYawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            left_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_back_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            headingError = Calculate_Heading_Error(currentYawAngle, targetAngleYaw);
            // 5 causes vacillation 6 does not 7 is safe
            if (Math.abs(headingError) < 7) {
                turnSpeed = Math.sqrt(Math.abs(headingError / 180));
            } else {
                turnSpeed = 0.5;
            }
            if (headingError < 0) {
                right_back_drive.setPower(-turnSpeed);
                left_front_drive.setPower(turnSpeed);
                left_back_drive.setPower(turnSpeed);
                right_front_drive.setPower(-turnSpeed);
            } else if (headingError > 0) {
                right_back_drive.setPower(turnSpeed);
                left_front_drive.setPower(-turnSpeed);
                left_back_drive.setPower(-turnSpeed);
                right_front_drive.setPower(turnSpeed);
            } else if (headingError == 0) {
                turnLoopCheckNumber += 1;
                if (turnLoopCheckNumber == 6) {
                    right_back_drive.setPower(0);
                    left_front_drive.setPower(0);
                    left_back_drive.setPower(0);
                    right_front_drive.setPower(0);
                    break;
                }
            }
        }
    }

    /**
     * Describe this function...
     */
   /* private void DriveForward_Back(double driveDistanceFeet) {
        DriveForward_BackAction(driveDistanceFeet);
        while (0 == 0) {
            sleep((long) (Math.abs(driveDistanceFeet) * 800));
            break;
        }
    }

    /**
     * Describe this function...
     */
/*private void ToggleBasket(
            // TODO: Enter the type for argument named Up_Down
            UNKNOWN_TYPE Up_Down) {
        if (Up_Down == "Up") {
            SBasket.setPosition(0.08);
        } else if (Up_Down == "Down") {
            SBasket.setPosition(0.12);
        } else {
            telemetry.addData("Error", "No basket position");
        }
    }

    /**
     * Describe this function...
     */
    /*private void HeadingErrorInitialization() {
        int target_heading;

        heading_tolerance = 0.5;
        target_heading = 0;
        headingError = 0;
    }

    /**
     * Describe this function...
     */
    /*private void InitializeIMU() {
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    /**
     * Describe this function...
     */
    /*private void DriveForward_BackAction(double driveDistanceFeetAction) {
        // TODO: Enter the type for variable named step
        UNKNOWN_TYPE step;

        left_back_drive.setTargetPosition(0);
        left_front_drive.setTargetPosition(0);
        right_back_drive.setTargetPosition(0);
        right_front_drive.setTargetPosition(0);
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Feet_To_Ticks_Calculation(driveDistanceFeetAction);
        if (Math.round(motorCurrentPositionAverage) > left_back_drive.getTargetPosition()) {
            right_back_drive.setPower(-DriveSpeed);
            left_front_drive.setPower(-DriveSpeed);
            left_back_drive.setPower(-DriveSpeed);
            right_front_drive.setPower(-DriveSpeed);
        } else if (Math.round(motorCurrentPositionAverage) < left_back_drive.getTargetPosition()) {
            right_back_drive.setPower(DriveSpeed);
            left_front_drive.setPower(DriveSpeed);
            left_back_drive.setPower(DriveSpeed);
            right_front_drive.setPower(DriveSpeed);
        } else if (Math.round(motorCurrentPositionAverage) == left_back_drive.getTargetPosition()) {
            right_back_drive.setPower(0);
            left_front_drive.setPower(0);
            left_back_drive.setPower(0);
            right_front_drive.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    /*private void InitializeMotors() {
        left_back_drive.setTargetPosition(0);
        left_front_drive.setTargetPosition(0);
        right_back_drive.setTargetPosition(0);
        right_front_drive.setTargetPosition(0);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCurrentPositionAverage = (left_back_drive.getCurrentPosition() + left_front_drive.getCurrentPosition() + right_back_drive.getCurrentPosition() + right_front_drive.getCurrentPosition()) / 4;
    }

    /**
     * Describe this function...
     */
    /*private int Calculate_Heading_Error(double orientation, int target_heading) {
        // Calculate heading_delta
        headingError = (int) (target_heading - currentYawAngle);
        // Calculate Bounded heading delta
        if (headingError < -180) {
            headingError += 360;
        } else if (headingError >= 180) {
            headingError += -360;
        } else {
            headingError += 0;
        }
        if (Math.abs(headingError) < heading_tolerance) {
            headingError = 0;
        }
        return headingError;
    }

    /**
     * Describe this function...
     */
    /*private void Feet_To_Ticks_Calculation(double distanceFeet) {
        double Ticks_Per_Rotation;
        double Circumference;
        double mmToFt;

        Ticks_Per_Rotation = 537.7;
        Circumference = 96 * Math.PI;
        mmToFt = 0.00328084;
        left_back_drive.setTargetPosition((int) (Ticks_Per_Rotation * distanceFeet * Circumference * mmToFt + left_back_drive.getCurrentPosition()));
        left_front_drive.setTargetPosition((int) (Ticks_Per_Rotation * distanceFeet * Circumference * mmToFt + left_front_drive.getCurrentPosition()));
        right_back_drive.setTargetPosition((int) (Ticks_Per_Rotation * distanceFeet * Circumference * mmToFt + right_back_drive.getCurrentPosition()));
        right_front_drive.setTargetPosition((int) (Ticks_Per_Rotation * distanceFeet * Circumference * mmToFt + right_front_drive.getCurrentPosition()));
    }
}*/
