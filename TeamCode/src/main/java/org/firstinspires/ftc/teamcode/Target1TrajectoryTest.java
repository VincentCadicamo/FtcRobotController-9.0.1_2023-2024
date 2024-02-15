package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Target 1", group = "TrajectorySequenceTest")
public class Target1TrajectoryTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    SampleMecanumDrive drive;
    Servo purpleServo;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        purpleServo = hardwareMap.get(Servo.class, "purplePixelServo");
        purpleServo.setPosition(.75);

        //Target1
        TrajectorySequence Target1 = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.00, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(12.00, -36.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-8.00, -36.00, Math.toRadians(180.00)))
                .lineTo(new Vector2d(51.00, -26.00))
                .addTemporalMarker(3.5, () -> {
                    purpleServo.setPosition(0);
                })
                .build();

        drive.setPoseEstimate(Target1.start());
        waitForStart();
        drive.followTrajectorySequence(Target1);
    }
}
