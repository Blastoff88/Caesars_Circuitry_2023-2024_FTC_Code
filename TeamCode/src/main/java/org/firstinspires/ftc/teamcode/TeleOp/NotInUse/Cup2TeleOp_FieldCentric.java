package org.firstinspires.ftc.teamcode.TeleOp.NotInUse;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Cup2TeleOp")
@Disabled
public class Cup2TeleOp_FieldCentric extends OpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor Hang;
    public Servo wall;
    public Servo Plane;

    public String speedFactor = "None";

    public static int WallDegrees;

    public boolean timeFlag = true;
    public boolean wallActive = false;
    public IMU imu;
    public double speedMultiplier = 1;
    public enum HangPos{
        EXTEND,
        RETRACT,
        OFF,
    }
    public HangPos GetHangPos = HangPos.OFF;

    public float lf_power = 0.0F;
    public float lb_power = 0.0F;
    public float rf_power = 0.0F;
    public float rb_power = 0.0F;



    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "M1");
        backLeft = hardwareMap.get(DcMotor.class, "M2");
        frontRight = hardwareMap.get(DcMotor.class, "M3");
        backRight = hardwareMap.get(DcMotor.class, "M4");
        Hang = hardwareMap.get(DcMotor.class, "Hang");
        wall = hardwareMap.get(Servo.class, "intakeServo");
        Plane = hardwareMap.get(Servo.class, "planeServo");
        imu= hardwareMap.get(IMU.class, "imu");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void loop() {
        telemetry.addData("Status", "Running");

        float x = gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;
        float turn = gamepad1.right_stick_x;

        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        float Nx = (float) (y * Math.sin(heading) + x * Math.cos(heading));// Might need to change negative sign based on IMU
        float Ny = (float) (y * Math.cos(heading) + x * Math.sin(heading));// Might need to change negative sign based on IMU
        FindJoystickMovement(Nx, Ny, turn);

        float leftFrontPower = lf_power;
        float leftBackPower = lb_power;
        float rightFrontPower = rf_power;
        float rightBackPower = rb_power;

        frontLeft.setPower(leftFrontPower);
        backLeft.setPower(leftBackPower);
        frontRight.setPower(rightFrontPower);
        backRight.setPower(rightBackPower);

        MoveSystems();

        AddTelemetry();
        telemetry.update();
    }

    public void FindJoystickMovement(float x, float y, float turn) {
        float theta = (float) Math.atan2(y, x);
        float power = (float) Math.hypot(x, y);

        float sin = (float) Math.sin(theta - Math.PI / 4);
        float cos = (float) Math.cos(theta - Math.PI / 4);
        float max = Math.max(Math.abs(sin), Math.abs(cos));

        lf_power = power * cos / max + turn;
        lb_power = power * sin / max + turn;
        rf_power = power * sin / max - turn;
        rb_power = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            lf_power /= power + turn;
            lb_power /= power + turn;
            rf_power /= power + turn;
            rb_power /= power + turn;
        }

    }

    public void MoveSystems() {
        if (gamepad2.left_trigger > 0 || gamepad1.left_trigger > 0) {
            wall.setPosition(.23);
        }
        if (gamepad2.right_trigger > 0 || gamepad1.right_trigger > 0) {
            wall.setPosition(.29);
        }
        if(gamepad2.left_bumper){
            Plane.setPosition(1);
        }

        if(gamepad2.right_bumper){
            Plane.setPosition(.7);

        }
        if(gamepad2.a){
            GetHangPos = HangPos.EXTEND;
        } else if (gamepad2.b) {
            GetHangPos = HangPos.RETRACT;
        } else if (gamepad2.x) {
            GetHangPos = HangPos.OFF;
        }
        if (GetHangPos == HangPos.EXTEND) {
            Hang.setTargetPosition(5000);
            Hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Hang.setPower(1);
        } else if (GetHangPos == HangPos.RETRACT) {
            Hang.setTargetPosition(0);
            Hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Hang.setPower(1);
        } else if (GetHangPos == HangPos.OFF) {
            Hang.setPower(0);
            Hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void AddTelemetry() {
        telemetry.addLine("Wheel Speed Factor");
        telemetry.addData("Speed Factor: ", speedFactor);

        telemetry.addLine("\n");
        telemetry.addData("flag",timeFlag);
        telemetry.addData("Time", System.currentTimeMillis());

        telemetry.addLine("Wheel Speeds");
        telemetry.addData("Left Front Power: ", lf_power * speedMultiplier);
        telemetry.addData("Left Back Power: ", lb_power * speedMultiplier);
        telemetry.addData("Right Front Power: ", rf_power * speedMultiplier);
        telemetry.addData("Right Back Power: ", rb_power * speedMultiplier);
    }
}