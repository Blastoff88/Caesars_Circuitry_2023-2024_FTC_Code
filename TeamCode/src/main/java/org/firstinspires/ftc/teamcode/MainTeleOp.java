package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class MainTeleOp extends OpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor arm;
    public DcMotor intake;
    public Servo intakePush;

    public static double armDegrees = 0.0;
    public double ticks_in_degrees = 537.7/360.0;

    public boolean isBoosted = false;

    public boolean intakeActive = false;

    public double speedMultiplier = 1;

    public double microMovementSpeed = 0.05;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "M2");
        backLeft = hardwareMap.get(DcMotor.class, "M1");
        frontRight = hardwareMap.get(DcMotor.class, "M4");
        backRight = hardwareMap.get(DcMotor.class, "M3");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakePush = hardwareMap.get(Servo.class, "intakeServo");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        intakePush.setPosition(1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop() {
        telemetry.addData("Status", "Running");
        telemetry.update();

        if (gamepad1.right_trigger > 0 || isBoosted) {
            Boost();
        }

        float x = -gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        float turn = -gamepad1.right_stick_x;

        float theta = (float)Math.atan2(y,x);
        float power = (float)Math.hypot(x,y);

        float sin = (float)Math.sin(theta -Math.PI/4);
        float cos = (float)Math.cos(theta -Math.PI/4);
        float max = (float)Math.max(Math.abs(sin), Math.abs(cos));

        float lf_power = power * cos/max + turn;
        float lb_power = power * sin/max + turn;
        float rf_power = power * sin/max - turn;
        float rb_power = power * cos/max - turn;

        if((power + Math.abs(turn)) > 1) {
            lf_power /= power + turn;
            lb_power /= power + turn;
            rf_power /= power + turn;
            rb_power /= power + turn;
        }

        frontLeft.setPower(lf_power * speedMultiplier);
        backLeft.setPower(lb_power * speedMultiplier);
        frontRight.setPower(rf_power * speedMultiplier);
        backRight.setPower(rb_power * speedMultiplier);

        telemetry.addData("Left Front Power: ",lf_power * speedMultiplier);
        telemetry.addData("Left Back Power: ",lb_power * speedMultiplier);
        telemetry.addData("Right Front Power: ",rf_power * speedMultiplier);
        telemetry.addData("Right Back Power: ",rb_power * speedMultiplier);

        MicroMovement();

        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            Intake();
        }

        Arm();
    }

    public void MicroMovement() {
        if (gamepad1.dpad_up) {
            frontLeft.setPower(microMovementSpeed);
            backLeft.setPower(microMovementSpeed);
            frontRight.setPower(microMovementSpeed);
            backRight.setPower(microMovementSpeed);
        }

        else if (gamepad1.dpad_right) {
            frontLeft.setPower(microMovementSpeed);
            backLeft.setPower(-microMovementSpeed);
            frontRight.setPower(-microMovementSpeed);
            backRight.setPower(microMovementSpeed);
        }

        else if (gamepad1.dpad_down) {
            frontLeft.setPower(-microMovementSpeed);
            backLeft.setPower(-microMovementSpeed);
            frontRight.setPower(-microMovementSpeed);
            backRight.setPower(-microMovementSpeed);
        }

        else if (gamepad1.dpad_left) {
            frontLeft.setPower(-microMovementSpeed);
            backLeft.setPower(microMovementSpeed);
            frontRight.setPower(microMovementSpeed);
            backRight.setPower(-microMovementSpeed);
        }
    }

    public void Boost() {
        if (gamepad1.right_trigger > 0 & !isBoosted) {
            isBoosted = true;
            speedMultiplier = 1;
        }
        else {
            isBoosted = false;
            speedMultiplier = 0.5;
        }

        telemetry.addData("Is Boosted: ", isBoosted);
    }

    public void Intake() {
        if (gamepad1.right_bumper) {
            intakeActive = true;
            intake.setPower(1.0);
            intakePush.setPosition(.5);
        }
        else if (gamepad1.left_bumper) {
            intakeActive = false;
            intake.setPower(0.0);
            intakePush.setPosition(0);
        }

        telemetry.addData("Intake Active: ", intakeActive);
    }

    public void Arm() {
        //Add arm code two options run to position or pidf controller
        //Run To Position Code
        arm.setPower(.005);
        arm.setTargetPosition((int)(ticks_in_degrees*armDegrees));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //PIDF Controller
    }
}