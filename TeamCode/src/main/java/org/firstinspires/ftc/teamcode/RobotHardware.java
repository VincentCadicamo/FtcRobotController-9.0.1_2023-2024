package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    //These motors are used to control the drivetrain
    public DcMotor LFMotor; //Left Front Motor
    public DcMotor LBMotor; //Left Back Motor
    public DcMotor RBMotor; //Right Back Motor
    public DcMotor RFMotor; //Right Front Motor

    //These motors are used to control the hanging mechanism
    public DcMotor hangingMotor1;
    public DcMotor hangingMotor2;

    //These motors control the Intake and Outtake Systems
    public DcMotor intakeMotor; //Motor that actuates the intake wheels
    public DcMotor linearSlideMotor; //Motor that actuates the linear slide

    public Servo airplaneLauncherServo;

    public Servo pixelHolderTiltServo1;
    public Servo pixelHolderTiltServo2;

    public Servo outtakeArmServo1;
    public Servo outtakeArmServo2;

    public Servo pixelDropServo;

    public Servo purplePixelServo;

    public Servo intakeRollerServo;

    //This is the onboard imu located on the controller hub
    public IMU imu;

    //This method initializes actuators and sensors
    public void init(HardwareMap hardwareMap) {

        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");

        intakeMotor = hardwareMap.get(DcMotor.class, "Emotor0");
        linearSlideMotor = hardwareMap.get(DcMotor.class, "Emotor1");
        hangingMotor1 = hardwareMap.get(DcMotor.class, "Emotor2");
        hangingMotor2 = hardwareMap.get(DcMotor.class, "Emotor3");

        airplaneLauncherServo = hardwareMap.get(Servo.class, "servo6");

        pixelHolderTiltServo1 = hardwareMap.get(Servo.class, "servo0");
        pixelHolderTiltServo2 = hardwareMap.get(Servo.class, "servo1");

        outtakeArmServo1 = hardwareMap.get(Servo.class, "servo2");
        outtakeArmServo2 = hardwareMap.get(Servo.class, "servo4");

        pixelDropServo = hardwareMap.get(Servo.class, "servo3");

        purplePixelServo = hardwareMap.get(Servo.class, "purplePixelServo");
        intakeRollerServo = hardwareMap.get(Servo.class, "servo7");



        LFMotor.setPower(0.0);
        LBMotor.setPower(0.0);
        RBMotor.setPower(0.0);
        RFMotor.setPower(0.0);

        intakeMotor.setPower(0.0);
        linearSlideMotor.setPower(0.0);
        hangingMotor1.setPower(0.0);
        hangingMotor2.setPower(0.0);

        airplaneLauncherServo.setPosition(1.0);

        pixelHolderTiltServo1.setPosition(.85);
        pixelHolderTiltServo2.setPosition(0.15);

        outtakeArmServo1.setPosition(0.0);
        outtakeArmServo2.setPosition(0.9);

        pixelDropServo.setPosition(0.0);
        purplePixelServo.setPosition(1.0);

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        hangingMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        hangingMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}