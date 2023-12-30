package org.firstinspires.ftc.teamcode.TeleOp.Cup1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class HangTest extends LinearOpMode {
    public DcMotor hang;
    public double topPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        hang = hardwareMap.get(DcMotor.class, "Hang");

        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Top position", topPosition);
            if (gamepad1.y) {
                topPosition = hang.getCurrentPosition();
            }

            if (gamepad1.a) {
                hang.setTargetPosition((int) topPosition);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(1);
            } else if (gamepad1.b) {
                hang.setTargetPosition(0);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(1);
            } //else if (hang.getCurrentPosition()<=1) {
//                hang.setPower(0);
//                hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
            telemetry.update();

        }
    }
}
