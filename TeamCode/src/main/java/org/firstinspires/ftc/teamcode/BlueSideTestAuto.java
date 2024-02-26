package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "Viper");
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(1);
                    lift.setTargetPosition(1000);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1000) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Claw {
        private Servo claw;



        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12,62,Math.toRadians(270)));
        Lift lift = new Lift(hardwareMap);
        // vision here that outputs position
        int visionOutputPosition = 1;

        Action blueLeft;
        Action blueCenter;
        Action blueRight;
        Action doubleLoop;

        blueLeft = drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(42,41,Math.PI),Math.toRadians(270))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(24,19),Math.PI)
                .build();
        blueCenter = drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(42,36,Math.PI),Math.toRadians(270))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(24,19),Math.PI)
                .build();
        blueRight = drive.actionBuilder(drive.pose)
                .splineToSplineHeading(new Pose2d(42,31,Math.PI),Math.toRadians(270))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(24,19),Math.PI)
                .build();
        doubleLoop = drive.actionBuilder(new Pose2d(24,19,Math.PI))
                .splineToConstantHeading(new Vector2d(28,9),Math.PI)
                .lineToXConstantHeading(-58)
                .waitSeconds(2)
                .lineToXConstantHeading(30)
                .splineToConstantHeading(new Vector2d(42,28),Math.PI)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(42,9),Math.PI)
                .lineToXConstantHeading(-58)
                .waitSeconds(2)
                .lineToXConstantHeading(30)
                .splineToConstantHeading(new Vector2d(42,28),Math.PI)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = blueCenter;
        } else if (startPosition == 2) {
            trajectoryActionChosen = blueLeft;
        } else {
            trajectoryActionChosen = blueRight;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        doubleLoop
                )
        );
    }
}