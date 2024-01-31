package org.firstinspires.ftc.teamcode.TeleOp.NotInUse;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Disabled
public class Tophat_Build_Day extends LinearOpMode{

    public Servo Wall;
    public DcMotor Hang;
    public Servo Plane;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Wall = hardwareMap.get(Servo.class, "Wall");
        Hang = hardwareMap.get(DcMotor.class, "Hang");
        Plane = hardwareMap.get(Servo.class, "Plane");
        Wall.setPosition(1);

        Hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hang.setPower(0);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            if(gamepad2.right_trigger>0){
                Wall.setPosition(1);
            } else if(gamepad2.left_trigger>0){
                Wall.setPosition(.7);
            }

            if(gamepad2.right_bumper){
                Plane.setPosition(.34);
            } else if (gamepad2.left_bumper) {
                Plane.setPosition(0);
            }

            if (gamepad2.a) {
                Hang.setTargetPosition(-5000);
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