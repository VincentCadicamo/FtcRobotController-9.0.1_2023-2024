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
    public DcMotor armMotor0; //Arm Pivot Motor
    public DcMotor armMotor1; //Arm Extension Motor

    public DcMotor liftMotor1; //Motor that actuates the spool on the line slide
    public DcMotor liftMotor2; //^

    public DcMotor outtake2;
    public DcMotor outtake3;

    public Servo airplane;
    public Servo rotate;
    public Servo clawServo;
    public Servo purplePixelServo;

    //This is the onboard imu located on the controller hub
    public IMU imu;

    //This method initializes actuators and sensors
    public void init(HardwareMap hardwareMap) {

        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        armMotor0 = hardwareMap.get(DcMotor.class, "armMotor0");
        armMotor1 = hardwareMap.get(DcMotor.class, "armMotor1");
        //TODO: set motor devices
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");
        outtake3 = hardwareMap.get(DcMotor.class, "outtake3");

        airplane = hardwareMap.get(Servo.class, "airplane");
        rotate = hardwareMap.get(Servo.class, "rotate");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        purplePixelServo = hardwareMap.get(Servo.class, "purplePixelServo");

        LFMotor.setPower(0.0);
        LBMotor.setPower(0.0);
        RBMotor.setPower(0.0);
        RFMotor.setPower(0.0);
        armMotor0.setPower(0.0);
        armMotor1.setPower(0.0);
        outtake2.setPower(0.0);
        outtake3.setPower(0.0);

        airplane.setPosition(1.0);
        rotate.setPosition(0.28);
        clawServo.setPosition(0.5);
        purplePixelServo.setPosition(0.7);

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        armMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        outtake3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}