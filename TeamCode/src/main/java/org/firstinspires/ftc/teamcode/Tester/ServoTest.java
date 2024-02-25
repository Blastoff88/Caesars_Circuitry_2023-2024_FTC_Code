package org.firstinspires.ftc.teamcode.Tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTest extends OpMode {
    public static double Position1; // what is the value you want it to turn
    public static double Position2;
    public Servo Servo;
    public static String Sname = ""; //Servos name
    @Override
    public void init() {
        Servo = hardwareMap.get(Servo.class, Sname);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void loop() {
        telemetry.addData("Status", "Running");
           if(gamepad1.a) {
               Servo.setPosition(Position1);
           } else if (gamepad1.b) {
               Servo.setPosition(Position2);
           }}
    }

