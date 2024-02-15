package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class TeleopV1 extends OpMode {

    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;
    private DcMotor RFMotor;

    private DcMotor armMotor0;
    private DcMotor armMotor1;

    private DcMotor outtake2;
    private DcMotor outtake3;

    private Servo airplane;
    private Servo rotate;
    private Servo clawServo;
    private Servo purplePixelServo;

    RobotHardware robot = new RobotHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        LFMotor = robot.LFMotor;
        LBMotor = robot.LBMotor;
        RBMotor = robot.RBMotor;
        RFMotor = robot.RFMotor;

        armMotor0 = robot.armMotor0;
        armMotor1 = robot.armMotor1;
        outtake2 = robot.outtake2;
        outtake3 = robot.outtake3;

        airplane = robot.airplane;
        rotate = robot.rotate;
        clawServo = robot.clawServo;
        purplePixelServo = robot.purplePixelServo;
    }

    @Override
    public void loop() {

        //Driver #1 Gamepad Controls
        //Buttons
        boolean G1xButton = gamepad1.x;
        boolean G1aButton = gamepad1.a;
        boolean G1bButton = gamepad1.b;
        boolean G1yButton = gamepad1.y;

        //Triggers
        double G1rightTrigger = gamepad1.right_trigger;
        double G1leftTrigger = gamepad1.left_trigger;

        //Sticks X and Y values
        double G1RightStickX = gamepad1.right_stick_x;
        double G1RightStickY = gamepad1.right_stick_y;

        double G1LeftStickX = gamepad1.left_stick_x;
        double G1LeftStickY = gamepad1.left_stick_y;

        //Driver #2 Gamepad Controls
        //Buttons
        boolean G2xButton = gamepad2.x;
        boolean G2aButton = gamepad2.a;
        boolean G2bButton = gamepad2.b;
        boolean G2yButton = gamepad2.y;

        //Triggers
        double G2rightTrigger = gamepad2.right_trigger;
        double G2leftTrigger = gamepad2.left_trigger;

        //Sticks X and Y values
        double G2RightStickX = gamepad2.right_stick_x;
        double G2RightStickY = gamepad2.right_stick_y;

        double G2LeftStickX = gamepad2.left_stick_x;
        double G2LeftStickY = gamepad2.left_stick_y;

        //Drivetrain Variables
        double LeftMotorBP = 0;
        double RightMotorBP = 0;
        double LeftMotorFP = 0;
        double RightMotorFP = 0;
        double VerticalPower = (G1rightTrigger - G1leftTrigger);

        //Logic
        RightMotorFP = ((VerticalPower - G1LeftStickX) - (G1RightStickX));
        LeftMotorFP = ((VerticalPower + G1LeftStickX) + (G1RightStickX));
        RightMotorBP = ((VerticalPower + G1LeftStickX) - (G1RightStickX));
        LeftMotorBP = ((VerticalPower - G1LeftStickX) + (G1RightStickX));

        LBMotor.setPower(-LeftMotorBP);
        RBMotor.setPower(RightMotorBP);
        LFMotor.setPower(-LeftMotorFP);
        RFMotor.setPower(RightMotorFP);

        if (G2aButton) {
            airplane.setPosition(-1.0);
        } else {
            airplane.setPosition(1.0);
        }

        if (G1aButton) {
            clawServo.setPosition(0);
        }
        if (G1bButton) {
            clawServo.setPosition(0.5);
        }

        if(G1yButton || G2yButton) {
            rotate.setPosition(0.55);
        }
        if(G1xButton || G2xButton) {
            rotate.setPosition(0.27);
        }

        if(gamepad1.dpad_down) {
            purplePixelServo.setPosition(0);
        }
        if(gamepad1.dpad_up) {
            purplePixelServo.setPosition(0.7);
        }


        armMotor0.setPower(-G1RightStickY);
        armMotor1.setPower(-G2RightStickY);

        outtake2.setPower(G2LeftStickY/2);
        telemetry.addData("Pos", armMotor0.getCurrentPosition());
    }
}
