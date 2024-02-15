package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OuttakeTest extends OpMode {

    Servo armServo; //Controller Hub port 2
    Servo dropServo; //Controller Hub port 3
    Servo rotateServo; //Controller Hub port 4

    double armServoPosition = 0.0;
    double dropServoPosition = 0.0;
    double rotateServoPosition = 0.0;

    @Override
    public void init() {
        armServo = hardwareMap.get(Servo.class, "armServo");
        dropServo = hardwareMap.get(Servo.class, "dropServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");

        armServo.setPosition(0);
        dropServo.setPosition(.1);
        rotateServo.setPosition(0);

    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            armServoPosition += 0.01;
        } else if (gamepad1.dpad_down){
            armServoPosition -= 0.01;
        }

        if(gamepad1.b) {
            rotateServoPosition += 0.01;
        } else if (gamepad1.a) {
            rotateServoPosition -= 0.01;
        }

        if(gamepad1.y) {
            dropServoPosition = 1;
        } else if (gamepad1.x){
            dropServoPosition = .1;
        }

        telemetry.addData("armServo", armServoPosition);
        telemetry.addData("rotateServo", rotateServoPosition);
        telemetry.addData("dropServo", dropServoPosition);
        armServo.setPosition(armServoPosition);
        rotateServo.setPosition(rotateServoPosition);
        dropServo.setPosition(dropServoPosition);
    }
}
