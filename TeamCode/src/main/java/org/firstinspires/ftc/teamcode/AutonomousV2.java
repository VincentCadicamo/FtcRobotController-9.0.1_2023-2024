package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedAutoBackDropArea", group = "Competition Code")
public class AutonomousV2 extends LinearOpMode {
    //Declare variables
    VisionManager visionManager;

    SampleMecanumDrive drive;
    Pose2d startPose;

    ElapsedTime timer;

    public Servo airplaneLauncherServo;
    public Servo pixelHolderTiltServo1;
    public Servo pixelHolderTiltServo2;
    public Servo outtakeArmServo1;
    public Servo outtakeArmServo2;
    public Servo pixelDropServo;
    public Servo purplePixelServo;
    public Servo cameraServo;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("(Odometry) Status:", "Initializing...");
        telemetry.update();

        //Initialize all variables
        drive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(12, -63, Math.toRadians(90.00));
        timer = new ElapsedTime();
        drive.setPoseEstimate(startPose);

        //Initialize TrajectorySequences
        TrajectorySequence Target1Left = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(11.94, -37.78))
                .lineToConstantHeading(new Vector2d(0.49, -37.50))
                //Drop Purple pixel
                .addDisplacementMarker(() -> {
                    purplePixelServo.setPosition(0);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(39.04, -30.24, Math.toRadians(180.00)))
                .addDisplacementMarker(() -> {
                    outtakeArmServo1.setPosition(0.8);
                    outtakeArmServo2.setPosition(0.1);

                    pixelHolderTiltServo2.setPosition(.45);
                    pixelHolderTiltServo2.setPosition(.55);
                })
                .lineToConstantHeading(new Vector2d(50.35, -29.68))
                .addDisplacementMarker(() -> {
                pixelDropServo.setPosition(0.0);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(22.14, -11.38))
                .lineToConstantHeading(new Vector2d(-59.01, -11.94))
                .lineToConstantHeading(new Vector2d(38.90, -11.80))
                .lineToConstantHeading(new Vector2d(44.48, -38.62))
                .lineToConstantHeading(new Vector2d(50.63, -38.62))
                .build();

        TrajectorySequence Target2Middle = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(11.94, -34.50))
                .lineToConstantHeading(new Vector2d(42.95, -36.52))
                .addDisplacementMarker(() -> {
                    purplePixelServo.setPosition(0);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(50.21, -36.52))
                .addDisplacementMarker(() -> {
                    outtakeArmServo1.setPosition(0.8);
                    outtakeArmServo2.setPosition(0.1);

                    pixelHolderTiltServo2.setPosition(.45);
                    pixelHolderTiltServo2.setPosition(.55);
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    pixelDropServo.setPosition(0.0);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence Target3Right = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(270.00)))
                .UNSTABLE_addTemporalMarkerOffset(11.90,() -> {})
                .lineToConstantHeading(new Vector2d(23.53, -42.95))
                .addDisplacementMarker(() -> {
                    purplePixelServo.setPosition(0);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(50.35, -42.81))
                .addDisplacementMarker(() -> {
                    outtakeArmServo1.setPosition(0.8);
                    outtakeArmServo2.setPosition(0.1);

                    pixelHolderTiltServo2.setPosition(.45);
                    pixelHolderTiltServo2.setPosition(.55);
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    pixelDropServo.setPosition(0.0);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(27.86, -12.64))
                .lineToConstantHeading(new Vector2d(-59.29, -12.08))
                .lineToConstantHeading(new Vector2d(33.03, -12.50))
                .lineToConstantHeading(new Vector2d(44.62, -36.66))
                .lineToConstantHeading(new Vector2d(50.35, -36.10))
                .build();


        telemetry.addData("(Odometry) Status:", "<Ready>");
        telemetry.update();

        visionManager = new VisionManager(hardwareMap, .41f, telemetry);

        airplaneLauncherServo = hardwareMap.get(Servo.class, "servo6");
        pixelHolderTiltServo1 = hardwareMap.get(Servo.class, "servo0");
        pixelHolderTiltServo2 = hardwareMap.get(Servo.class, "servo1");
        outtakeArmServo1 = hardwareMap.get(Servo.class, "servo2");
        outtakeArmServo2 = hardwareMap.get(Servo.class, "servo4");
        pixelDropServo = hardwareMap.get(Servo.class, "servo3");
        cameraServo = hardwareMap.get(Servo.class, "servo5");
        purplePixelServo = hardwareMap.get(Servo.class, "purplePixelServo");

        cameraServo.setPosition(.35);

//        purplePixelServo.setPosition(.75);
//        pixelHolderTiltServo1.setPosition(.35);
//        pixelHolderTiltServo2.setPosition(0);
//        outtakeArmServo1.setPosition(0.85);
//        outtakeArmServo2.setPosition(0.15);
//        pixelDropServo.setPosition(0.0); //1.0 to drop

        waitForStart();

        while (opModeIsActive()) {
            boolean targetFound = false;
            int targetPosition = 0;

            //TODO: Camera starting position set
            //Turn camera towards each pixel, end condition if team element found 3 seconds used out of 30;
            breakcondition:
            while (!targetFound) {
                int currentIteration = 0;
                currentIteration += 1;
                telemetry.addData("(CameraDetection) Status:", "<Iteration> " + currentIteration + "\n<Current Target> 1");
                telemetry.update();
                //Determine Team Element on position 1
                timer.reset();
                while (timer.milliseconds() < 1300) {
                    cameraServo.setPosition(0.5);
                    if (visionManager.determineTeamElement()) {
                        targetPosition = 1;
                        targetFound = true;
                        break breakcondition;
                    }
                    telemetry.update();
                }
                //Determine Team Element on position 2
                telemetry.addData("(CameraDetection) Status:", "<Iteration> " + currentIteration + "\n<Current Target> 2");
                telemetry.update();
                timer.reset();
                while (timer.milliseconds() < 1300) {
                    cameraServo.setPosition(0.65);
                    if (visionManager.determineTeamElement()) {
                        targetPosition = 2;
                        targetFound = true;
                        break breakcondition;
                    }
                    telemetry.update();
                }
                //Determine Team Element on position 3
                telemetry.addData("(CameraDetection) Status:", "<Iteration> " + currentIteration + "\n<Current Target> 3");
                telemetry.update();
                timer.reset();
                while (timer.milliseconds() < 1300) {
                    cameraServo.setPosition(.80);
                    if (visionManager.determineTeamElement()) {
                        targetPosition = 3;
                        targetFound = true;
                        break breakcondition;
                    }
                    telemetry.update();
                }
            }

            //TODO: Place Purple Pixel on corresponding tape line
            //Move over tap and actuate purple pixel placer.
            //TODO: Place Yellow Pixel on BackDrop
            //Depending on the Pixel Detection place the yellow pixel on backdrop in correct spot
            //Same spline path.
            switch (targetPosition) {
                case 0:
                case 1:
                    drive.followTrajectorySequence(Target1Left);
                    break;
                case 2:
                    drive.followTrajectorySequence(Target2Middle);
                    break;
                case 3:
                    drive.followTrajectorySequence(Target3Right);
                    break;
            }

            //TODO: Grab white pixels from back to place on backdrop
            //Using Road Runner move to front stage area and take two white pixels,
            //Then move robot back to the back drop and drop the pixels
            //Most likely repeat until time runs out
            //Probably out of our capabilities unless we find a way for perfect linear movement of claw, most likely a crank mechanism

        }
    }
}