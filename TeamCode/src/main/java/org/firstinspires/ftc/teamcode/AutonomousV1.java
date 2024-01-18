package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class AutonomousV1 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    VisionManager visionManager;

    @Override
    public void runOpMode() throws InterruptedException {
        visionManager = new VisionManager(hardwareMap, .75f, telemetry);
        waitForStart();
        while (opModeIsActive()) {

        }

    }
}