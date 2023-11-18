package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

    @Config
    @TeleOp
    public class ScrimmageTeleOp extends OpMode {
        public DcMotor frontLeft;
        public DcMotor backLeft;
        public DcMotor frontRight;
        public DcMotor backRight;
        public Servo wall;

        public String speedFactor = "None";

        public static int WallDegrees;

        public boolean timeFlag = true;
        public boolean wallActive = false;

        public double speedMultiplier = 1;

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
            wall = hardwareMap.get(Servo.class, "intakeServo");

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }

        public void loop() {
            telemetry.addData("Status", "Running");

            float x = -gamepad1.left_stick_x;
            float y = gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x / 2;

            FindJoystickMovement(x, y, turn);

            float leftFrontPower = lf_power;
            float leftBackPower = lb_power;
            float rightFrontPower = rf_power;
            float rightBackPower = rb_power;


            frontLeft.setPower(leftFrontPower * speedMultiplier);
            backLeft.setPower(leftBackPower * speedMultiplier);
            frontRight.setPower(rightFrontPower * speedMultiplier);
            backRight.setPower(rightBackPower * speedMultiplier);

                MoveArm();

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
            System.currentTimeMillis();

        }

        public void MoveArm() {
            if (timeFlag == true) {
                if (gamepad1.left_trigger > 0 & !wallActive) {
                    wall.setPosition(.23);
                    time = System.currentTimeMillis();
                    timeSleep(5);
                }
                if (gamepad1.left_trigger > 0 & wallActive) {
                    wall.setPosition(.29);
                    time = System.currentTimeMillis();
                    timeSleep(500);
                }
            }
        }

        public void timeSleep(double milliseconds) {
            timeFlag = false;
            if (time == (time + milliseconds)) {
                timeFlag = true;
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

