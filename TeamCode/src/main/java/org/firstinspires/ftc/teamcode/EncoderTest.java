package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class EncoderTest extends OpMode {

    private DcMotor leftEncoder;
    private DcMotor rightEncoder;

    @Override
    public void init() {
        leftEncoder = (hardwareMap.get(DcMotor.class, "outtake2"));
        rightEncoder = (hardwareMap.get(DcMotor.class, "outtake3"));

    }

    @Override
    public void loop() {
        int leftEncoderPos = leftEncoder.getCurrentPosition();
        int rightEncoderPos = rightEncoder.getCurrentPosition();

        telemetry.addData("Left: ", leftEncoderPos);
        telemetry.addData("Right: ", rightEncoderPos);
    }
}
