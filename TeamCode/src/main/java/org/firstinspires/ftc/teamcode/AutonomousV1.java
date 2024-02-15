package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedAutoBackDropArea", group = "Competition Code")
public class AutonomousV1 extends LinearOpMode {
    //Declare variables
    VisionManager visionManager;

    SampleMecanumDrive drive;
    Pose2d startPose;

    ElapsedTime timer;

    Servo cameraServo; //Controller Hub port 6
    Servo purpleServo; //Controller Hub port 0
    Servo armServo; //Controller Hub port 3
    Servo dropServo; //Controller Hub port 4
    Servo rotateServo; //Controller Hub port 5

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
        TrajectorySequence Target1Left = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(12.00, -36.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-8.00, -36.00, Math.toRadians(180.00))) //Change -8 to 6 if droping purple pixel from intake wheels
                .lineTo(new Vector2d(51.00, -26.00))
                .addTemporalMarker(3.5, () -> {
                    //Drop purple pixel on tape
                    purpleServo.setPosition(0);
                })
                .build();

        TrajectorySequence Target2Middle = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(12.00, -33.00), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(11.83, -36.00))
                .lineToSplineHeading(new Pose2d(50.60, -36.00, Math.toRadians(180.00)))
                .build();

        TrajectorySequence Target3Right = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(23.57, -42.94), Math.toRadians(90.00))
                .lineToSplineHeading(new Pose2d(50.60, -42.75, Math.toRadians(180.00)))
                .build();

        telemetry.addData("(Odometry) Status:", "<Ready>");
        telemetry.update();

        visionManager = new VisionManager(hardwareMap, .75f, telemetry);

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        purpleServo = hardwareMap.get(Servo.class, "purplePixelServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
        dropServo = hardwareMap.get(Servo.class, "dropServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");

        purpleServo.setPosition(.75);
        cameraServo.setPosition(.35);
        armServo.setPosition(0);
        dropServo.setPosition(0);
        rotateServo.setPosition(0);

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
                    cameraServo.setPosition(0.35);
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
                    cameraServo.setPosition(0.5);
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
                    cameraServo.setPosition(.65);
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