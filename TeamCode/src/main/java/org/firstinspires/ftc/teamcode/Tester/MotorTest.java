package org.firstinspires.ftc.teamcode.Tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class MotorTest extends OpMode {
    public DcMotor Motor;
//    public DcMotor Motor2;
//    public DcMotor Motor3;
    public static String Mname =""; // motors name
//    public static String Mname2 ="";
//    public static String Mname3 ="";
    public static boolean HasEncoder = false; // does it have an encoder
    public static boolean runToPos = false; // do you want it to run to position
    public static  boolean RunUsingEncoder = false; // do you want it to adjust for its self
    public static  double Power = 0; // motor power
    public static double Power2 = 0;
    public static double Power3 =0;
    public static  int Position = 0; // Motor position if using run to pos
    @Override
    public void init() {
        Motor = hardwareMap.get(DcMotor.class, Mname);
//        Motor2 = hardwareMap.get(DcMotor.class,Mname2);
//        Motor3 = hardwareMap.get(DcMotor.class,Mname3);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Motor.setTargetPosition(0);
        if(HasEncoder){ // If motor has an encoder
            if(RunUsingEncoder){ // if you want it to adjust using encoder values
                Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (runToPos) { // if you want it to run to position
                Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else { // if motor has encoder but don't want it to adjust
                Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }
    public void loop() {
        telemetry.addData("Status", "Running");
            while(!runToPos){ // While the mode is not run To pos
                Motor.setPower(Power);
//                Motor2.setPower(Power2);
            }
                while (runToPos) { // while it is runTo Pos
//                    Motor2.setPower(Power2);
//                    Motor3.setPower(Power3);
                    Motor.setPower(Power);
                    Motor.setTargetPosition(Position);
                }
            }

        }


