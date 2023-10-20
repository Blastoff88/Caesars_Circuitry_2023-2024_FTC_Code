package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MecanumDrivetrain extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor arm;
    public DcMotor intake;

    public boolean isBoosted = false;

    public boolean bPressed = false;

    public double speedMultiplier = 0.75;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "M2");
        backLeft = hardwareMap.get(DcMotor.class, "M1");
        frontRight = hardwareMap.get(DcMotor.class, "M4");
        backRight = hardwareMap.get(DcMotor.class, "M3");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
//Need to flip polarity of motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            float x = gamepad1.left_stick_x;
            float y = gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x;

            float theta = (float)Math.atan2(y,x);
            float power = (float)Math.hypot(x,y);

            float sin = (float)Math.sin(theta -Math.PI/4);
            float cos = (float)Math.cos(theta -Math.PI/4);
            float max = Math.max(Math.abs(sin), Math.abs(cos)); // I deleted float b/c it is redundant

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

            if (gamepad1.a & !isBoosted) {
                isBoosted = true;
                speedMultiplier = 1;
            }
            else if (gamepad1.a & isBoosted) {
                isBoosted = false;
                speedMultiplier = 0.75;
            }

            if (gamepad1.b & !bPressed) {
                bPressed = true;
                intake.setPower(1.0);
            }
            else if (gamepad1.b & bPressed) {
                bPressed = false;
                intake.setPower(0.0);
            }

            if(gamepad1.dpad_down) {
               frontLeft.setPower(.2);
                frontRight.setPower(.2);
                backRight.setPower(.2);
                backLeft.setPower(.2);
            }
            if(gamepad1.dpad_up) {
                frontLeft.setPower(-.2);
                frontRight.setPower(-.2);
                backRight.setPower(-.2);
                backLeft.setPower(-.2);
            }
            if(gamepad1.dpad_left) {
                frontLeft.setPower(.2);
                frontRight.setPower(-.2);
                backRight.setPower(-.2);
                backLeft.setPower(.2);

            }
            if(gamepad1.dpad_right) {
                frontLeft.setPower(-.2);
                frontRight.setPower(.2);
                backRight.setPower(.2);
                backLeft.setPower(-.2);

            }

            frontLeft.setPower(lf_power * speedMultiplier);
            backLeft.setPower(lb_power * speedMultiplier);
            frontRight.setPower(rf_power * speedMultiplier);
            backRight.setPower(rb_power * speedMultiplier);
            telemetry.addData("Left Front Power: ",lf_power);
            telemetry.addData("Left Back Power: ",lb_power);
            telemetry.addData("Right Front Power: ",rf_power);
            telemetry.addData("Right Back Power: ",rb_power);


            telemetry.addData("isBoosted: ", isBoosted);


        }
    }
}