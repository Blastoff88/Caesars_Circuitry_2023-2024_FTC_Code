package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
    @TeleOp
    public class Cup1TeleOp extends OpMode {
        public DcMotor frontLeft;
        public DcMotor backLeft;
        public DcMotor frontRight;
        public DcMotor backRight;
        public DcMotor Hang;

        //public IMU imu;

        public Servo wall;
        public Servo Plane;

        public String speedFactor = "None";

        public static int WallDegrees;

        public boolean timeFlag = true;
        public boolean wallActive = false;

        public double speedMultiplier = 1;

        public float lf_power = 0.0F;
        public float lb_power = 0.0F;
        public float rf_power = 0.0F;
        public float rb_power = 0.0F;
    public double topPosition = 0;
    private ElapsedTime runtime = new ElapsedTime();
    public boolean isOverriden = false;


        @Override
        public void init() {
            frontLeft = hardwareMap.get(DcMotor.class, "M1");
            backLeft = hardwareMap.get(DcMotor.class, "M2");
            frontRight = hardwareMap.get(DcMotor.class, "M3");
            backRight = hardwareMap.get(DcMotor.class, "M4");
            Hang = hardwareMap.get(DcMotor.class, "Hang");
            wall = hardwareMap.get(Servo.class, "intakeServo");
            Plane = hardwareMap.get(Servo.class, "planeServo");

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            /*frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//            ));
//
//            imu.initialize(parameters);
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
                if (gamepad2.left_trigger > 0 || gamepad1.left_trigger > 0) {
                    wall.setPosition(.23);
                }
                if (gamepad2.right_trigger > 0 || gamepad1.right_trigger > 0) {
                    wall.setPosition(.29);
                }
                if(gamepad2.x && !isOverriden){
                    isOverriden = true;
                } else if (gamepad2.x && isOverriden) {
                    isOverriden = false;
                }

            if(getRuntime()>=90 || isOverriden){
                    if(gamepad2.left_bumper){
                        Plane.setPosition(1);

                    }

                    if(gamepad2.right_bumper){
                        Plane.setPosition(.7);

                    }
                    if(gamepad2.left_stick_y !=0){
                        Hang.setPower(gamepad2.left_stick_y);
                    }
                if (gamepad1.y) {
                    topPosition = hang.getCurrentPosition();
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

