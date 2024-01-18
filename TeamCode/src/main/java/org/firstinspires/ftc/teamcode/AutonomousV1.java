package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutonomousV1 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    VisionManager visionManager;
    SampleMecanumDrive drive;
    Pose2d startPose;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("(Odometry) Status:", "Initializing...");
        drive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(0, 0, 0);
        timer = new ElapsedTime();
        drive.setPoseEstimate(startPose);
        telemetry.addData("(Odometry) Status:", "<Ready>");
        telemetry.update();

        visionManager = new VisionManager(hardwareMap, .75f, telemetry);
        waitForStart();

        while (opModeIsActive()) {

            //TODO: Camera starting position set
            //Maybe move forward, up to full range for viewing pixels?

            //TODO: Camera Pixel Detection
            //Detect where on screen the pixel is eg. Left/Middle/Right

            //TODO: Place Purple Pixel on corresponding tape line
            //Move over tap and actuate purple pixel placer.

            //TODO: Place Yellow Pixel on BackDrop
            //Depending on the Pixel Detection place the yellow pixel on backdrop in correct spot

            //TODO: Grab white pixels from back to place on backdrop
            //Using Road Runner move to front stage area and take two white pixels,
            //Then move robot back to the back drop and drop the pixels
            //Most likely repeat until time runs out

            //TODO: WIN LEAGUE TOURNAMENT

        }

    }
}