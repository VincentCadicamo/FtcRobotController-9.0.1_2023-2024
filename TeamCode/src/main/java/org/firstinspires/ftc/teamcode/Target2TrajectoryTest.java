package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Target 2", group = "TrajectorySequenceTest")
public class Target2TrajectoryTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    SampleMecanumDrive drive;
    Servo purpleServo;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        purpleServo = hardwareMap.get(Servo.class, "purplePixelServo");
        purpleServo.setPosition(.75);

        //Target2
        TrajectorySequence Target2Middle = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(12.00, -33.00), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(11.83, -36.00))
                .lineToSplineHeading(new Pose2d(50.60, -36.00, Math.toRadians(180.00)))
                .build();

        drive.setPoseEstimate(Target2Middle.start());
        waitForStart();
        drive.followTrajectorySequence(Target2Middle);
    }
}
