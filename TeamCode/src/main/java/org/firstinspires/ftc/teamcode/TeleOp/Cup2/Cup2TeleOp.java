package org.firstinspires.ftc.teamcode.TeleOp.Cup2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class Cup2TeleOp extends LinearOpMode{

    public Servo Wall;
    public DcMotor Hang;
    public Servo Plane;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public static int ViperMax = -8000;
    public int ViperTruss = -5000;

    public float lf_power = 0.0F;
    public float lb_power = 0.0F;
    public float rf_power = 0.0F;
    public float rb_power = 0.0F;

    @Override
    public void runOpMode(){

        frontLeft = hardwareMap.get(DcMotorEx.class, "M3");
        backLeft = hardwareMap.get(DcMotorEx.class, "M4");
        backRight = hardwareMap.get(DcMotorEx.class, "M2");
        frontRight = hardwareMap.get(DcMotorEx.class, "M1");

        Wall = hardwareMap.get(Servo.class, "Wall");
        Hang = hardwareMap.get(DcMotor.class, "Hang");
        Plane = hardwareMap.get(Servo.class, "Plane");
        Wall.setPosition(1);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hang.setPower(0);
        waitForStart();


        while (!isStopRequested()) {
            float x = -gamepad1.left_stick_x;
            float y = gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x / (float)1.5;

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

            float leftFrontPower = lf_power;
            float leftBackPower = lb_power;
            float rightFrontPower = rf_power;
            float rightBackPower = rb_power;


            frontLeft.setPower(leftFrontPower);
            backLeft.setPower(leftBackPower);
            frontRight.setPower(rightFrontPower);
            backRight.setPower(rightBackPower);

            if(gamepad2.right_trigger>0){
                Wall.setPosition(1);
            } else if(gamepad2.left_trigger>0){
                Wall.setPosition(.7);
            }

            if(gamepad2.right_bumper){
                Plane.setPosition(.34);
            } else if (gamepad2.left_bumper && gamepad2.y) {
                Plane.setPosition(0);
            }

            if (gamepad2.a) {
                Hang.setTargetPosition(ViperMax);
                Hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Hang.setPower(1);
            } else if (gamepad2.b) {
                Hang.setTargetPosition(100);
                Hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Hang.setPower(1);
            } else if (gamepad2.x) {
                Hang.setPower(0);
                Hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }
}