package org.firstinspires.ftc.teamcode.PreviewDay;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class PreviewDayTeleOp extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor arm;
    public Servo intakeServo;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "M2");
        backLeft = hardwareMap.get(DcMotor.class, "M1");
        frontRight = hardwareMap.get(DcMotor.class, "M4");
        backRight = hardwareMap.get(DcMotor.class, "M3");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            float x = -gamepad1.left_stick_x;
            float y = gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x;

            float theta = (float) Math.atan2(y, x);
            float power = (float) Math.hypot(x, y);

            float sin = (float) Math.sin(theta - Math.PI / 4);
            float cos = (float) Math.cos(theta - Math.PI / 4);
            float max = (float) Math.max(Math.abs(sin), Math.abs(cos));

            float lf_power = power * cos / max + turn;
            float lb_power = power * sin / max + turn;
            float rf_power = power * sin / max - turn;
            float rb_power = power * cos / max - turn;

            if ((power + Math.abs(turn)) > 1) {
                lf_power /= power + turn;
                lb_power /= power + turn;
                rf_power /= power + turn;
                rb_power /= power + turn;
            }

            if (gamepad1.right_bumper) {
                intakeServo.setPosition(0.5);
            } else if (gamepad1.left_bumper) {
                intakeServo.setPosition(0.0);
            }

            frontLeft.setPower(lf_power);
            backLeft.setPower(lb_power);
            frontRight.setPower(rf_power);
            backRight.setPower(rb_power);
            telemetry.addData("Left Front Power: ", lf_power);
            telemetry.addData("Left Back Power: ", lb_power);
            telemetry.addData("Right Front Power: ", rf_power);
            telemetry.addData("Right Back Power: ", rb_power);
        }
    }
}