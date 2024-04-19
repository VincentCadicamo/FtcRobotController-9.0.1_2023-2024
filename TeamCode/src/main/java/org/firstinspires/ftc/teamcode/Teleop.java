package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Teleop extends OpMode {

    final double PixelHolderTiltServoPos1IntakePos = .8;
    final double PixelHolderTiltServoPos2IntakePos = .2;

    final double OuttakeArmServoPos1IntakePos = .9;
    final double OuttakeArmServoPos2IntakePos = .0;

    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;
    private DcMotor RFMotor;

    private DcMotor intakeMotor;
    private DcMotor linearSlideMotor;

    private DcMotor hangingMotor1;
    private DcMotor hangingMotor2;

    private Servo airplaneLauncherServo;

    private Servo pixelHolderTiltServo1;
    private Servo pixelHolderTiltServo2;

    private Servo outtakeArmServo1;
    private Servo outtakeArmServo2;

    private Servo pixelDropServo;

    RobotHardware robot = new RobotHardware();

    double outtakeArmServoPos1 = .0;
    double outtakeArmServoPos2 = .9;

    double pixelHolderTiltServoPos1 = .8;
    double pixelHolderTiltServoPos2 = .2;

    @Override
    public void init() {
        robot.init(hardwareMap);
        LFMotor = robot.LFMotor;
        LBMotor = robot.LBMotor;
        RBMotor = robot.RBMotor;
        RFMotor = robot.RFMotor;

        intakeMotor = robot.intakeMotor;
        linearSlideMotor = robot.linearSlideMotor;
        hangingMotor1 = robot.hangingMotor1;
        hangingMotor2 = robot.hangingMotor2;

        airplaneLauncherServo = robot.airplaneLauncherServo;

        pixelHolderTiltServo1 = robot.pixelHolderTiltServo1;
        pixelHolderTiltServo2 = robot.pixelHolderTiltServo2;
        outtakeArmServo1 = robot.outtakeArmServo1;
        outtakeArmServo2 = robot.outtakeArmServo2;

        pixelDropServo = robot.pixelDropServo;
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

        linearSlideMotor.setPower(G1LeftStickY);
        intakeMotor.setPower(-G2RightStickY * .80);

        if (G1aButton) {
            outtakeArmServoPos1 = 0.0;
            outtakeArmServoPos2 = 0.9;

            pixelHolderTiltServoPos1 = .8;
            pixelHolderTiltServoPos2 = .2;
        }

        if (G1bButton) {
            outtakeArmServoPos1 = 0.8;
            outtakeArmServoPos2 = 0.1;

            pixelHolderTiltServoPos1 = 0.45;
            pixelHolderTiltServoPos2 = 0.55;
        }

        if (G1yButton) {
            pixelDropServo.setPosition(1.0);
        } else {
            pixelDropServo.setPosition(0.0);
        }

        if (G2aButton) {
            robot.airplaneLauncherServo.setPosition(0.0);
        }
        if (G2xButton)
            robot.airplaneLauncherServo.setPosition(1.0);
        if (G2yButton) {
            robot.purplePixelServo.setPosition(0.0);
        }

        if (gamepad1.dpad_up) {
            robot.intakeRollerServo.setPosition(0.0);
        }
        if (gamepad1.dpad_down) {
            robot.intakeRollerServo.setPosition(1.0);
        }

        outtakeArmServo1.setPosition(outtakeArmServoPos1);
        outtakeArmServo2.setPosition(outtakeArmServoPos2);

        pixelHolderTiltServo1.setPosition(pixelHolderTiltServoPos1);
        pixelHolderTiltServo2.setPosition(pixelHolderTiltServoPos2);

        hangingMotor1.setPower(G2LeftStickY);
        hangingMotor2.setPower(G2LeftStickY);
    }
}