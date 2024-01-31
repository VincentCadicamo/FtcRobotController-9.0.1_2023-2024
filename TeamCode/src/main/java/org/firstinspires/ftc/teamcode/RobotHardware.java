package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.IMU;

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
        liftMotor1 = hardwareMap.get(DcMotor.class, "");
        liftMotor2 = hardwareMap.get(DcMotor.class, "");

        LFMotor.setPower(0.0);
        LBMotor.setPower(0.0);
        RBMotor.setPower(0.0);
        RFMotor.setPower(0.0);
        armMotor0.setPower(0.0);
        armMotor1.setPower(0.0);
        liftMotor1.setPower(0.0);
        liftMotor2.setPower(0.0);

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        armMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
    }
}