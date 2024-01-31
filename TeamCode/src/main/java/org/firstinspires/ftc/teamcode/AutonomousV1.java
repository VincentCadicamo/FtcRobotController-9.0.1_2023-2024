package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonomousV1 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    VisionManager visionManager;
    SampleMecanumDrive drive;
    Pose2d startPose;
    ElapsedTime timer;

    Servo cameraServo;
    Servo purpleServo;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("(Odometry) Status:", "Initializing...");

        drive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(0, 0, 0);
        timer = new ElapsedTime();
        drive.setPoseEstimate(startPose);
        TrajectorySequence[] trajectorySequences = new TrajectorySequence[4];

        //TODO: Initialize trajectories in init state
        TrajectorySequence moveToTape = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(12.00, -49.85), Math.toRadians(89.72))
                .build();
        drive.setPoseEstimate(moveToTape.start());

        TrajectorySequence target1;
        TrajectorySequence target2;
        TrajectorySequence target3;

        trajectorySequences[0] = moveToTape;
        //TODO: Fix target trajectory sequences
        /*
        trajectorySequences[1] = target1;
        trajectorySequences[2] = target2;
        trajectorySequences[3] = target3;
        */
        telemetry.addData("(Odometry) Status:", "<Ready>");
        telemetry.update();

        visionManager = new VisionManager(hardwareMap, .75f, telemetry);
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        purpleServo = hardwareMap.get(Servo.class, "purpleServo");

        purpleServo.setPosition(.75);
        cameraServo.setPosition(.35);
        waitForStart();

        while (opModeIsActive()) {
            boolean targetFound = false;
            int targetPosition;
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
                }
                //Determine Team Element on position 2
                telemetry.addData("(CameraDetection) Status:", "<Iteration> " + currentIteration + "\n<Current Target> 2");
                telemetry.update();
                timer.reset();
                while (timer.milliseconds() < 1300) {
                    cameraServo.setPosition(0.0);
                    if (visionManager.determineTeamElement()) {
                        targetPosition = 2;
                        targetFound = true;
                        break breakcondition;
                    }
                }
                //Determine Team Element on position 3
                telemetry.addData("(CameraDetection) Status:", "<Iteration> " + currentIteration + "\n<Current Target> 3");
                telemetry.update();
                timer.reset();
                while (timer.milliseconds() < 1300) {
                    cameraServo.setPosition(-0.35);
                    if (visionManager.determineTeamElement()) {
                        targetPosition = 3;
                        targetFound = true;
                        break breakcondition;
                    }
                }
            }

            //TODO: Place Purple Pixel on corresponding tape line
            //Move over tap and actuate purple pixel placer.
            //TODO: Place Yellow Pixel on BackDrop
            //Depending on the Pixel Detection place the yellow pixel on backdrop in correct spot
            //Same spline path.


            //TODO: Grab white pixels from back to place on backdrop
            //Using Road Runner move to front stage area and take two white pixels,
            //Then move robot back to the back drop and drop the pixels
            //Most likely repeat until time runs out
            //Probably out of our capabilities unless we find a way for perfect linear movement of claw, most likely a crank mechanism

            //TODO: WIN LEAGUE TOURNAMENT

        }

    }
}