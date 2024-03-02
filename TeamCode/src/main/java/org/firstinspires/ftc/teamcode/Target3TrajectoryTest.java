package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Target 3", group = "TrajectorySequenceTest")
public class Target3TrajectoryTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    SampleMecanumDrive drive;
    Servo purpleServo;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        purpleServo = hardwareMap.get(Servo.class, "purplePixelServo");
        purpleServo.setPosition(.75);

        //Target 3
        TrajectorySequence Target3Right = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(23.57, -42.94))
                .lineToSplineHeading(new Pose2d(46.00, -42.75, Math.toRadians(180.00)))
                .lineToConstantHeading(new Vector2d(34.99, -11.84))
                .lineToConstantHeading(new Vector2d(-61.38, -11.83))
                .build();

        drive.setPoseEstimate(Target3Right.start());
        waitForStart();
        drive.followTrajectorySequence(Target3Right);
    }
}
