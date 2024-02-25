package org.firstinspires.ftc.teamcode.Tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ViperTester extends OpMode {
    public DcMotor Motor;
    public DcMotor Motor2;
    public DcMotor Motor3;
    public Servo Servo1;
    public Servo Servo2;
    public static String Mname =""; // motors name
    public static String Mname2 ="";
    public static String Mname3 ="";
    public static String Sname1 = "";
    public static String Sname2 = "";
    public static boolean HasEncoder = false; // does it have an encoder
    public static boolean runToPos = false; // do you want it to run to position
    public static  boolean RunUsingEncoder = false; // do you want it to adjust for its self
    public static  double Power = 0; // motor power
    public static double Power2 = 0;
    public static double Power3 =0;
    public static double Spow1 = 0;
    public static double Spow2 = 0;
    public static  int Position = 0; // Motor position if using run to pos
    @Override
    public void init() {
        Motor = hardwareMap.get(DcMotor.class, Mname);
        Servo1 = hardwareMap.get(Servo.class, Sname1);
        Servo2 = hardwareMap.get(Servo.class, Sname2);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }
    public void loop() {
        telemetry.addData("Status", "Running");
            while(!runToPos){ // While the mode is not run To pos
                Motor.setPower(Power);
                if(gamepad1.a) {
                    Servo1.setPosition(Spow1);
                }
                if (gamepad1.b){
                    Servo2.setPosition(Spow2);
                }
            }
                while (runToPos) { // while it is runTo Pos

                    if(gamepad1.a){
                        Servo2.setPosition(Spow2);
                    }
                }
            }

        }


