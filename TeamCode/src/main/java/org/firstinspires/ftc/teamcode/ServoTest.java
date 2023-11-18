package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTest extends OpMode {
    public static int Degree;
    public Servo wall;
    @Override
    public void init() {
        wall = hardwareMap.get(Servo.class, "intakeServo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void loop() {
        telemetry.addData("Status", "Running");
            wall.setPosition(gamepad1.left_stick_y);

        }
    }

