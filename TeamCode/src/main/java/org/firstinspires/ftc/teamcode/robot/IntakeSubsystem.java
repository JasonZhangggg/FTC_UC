package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSubsystem {
    LinearOpMode op_mode = null;
    public DcMotor m0 = null;   // upper motor driving Geko wheels
    public DcMotor m1 = null;   // lower motor driving disc wheels
    public Servo dropper1 = null;
    public Servo dropper2 = null;
    HardwareMap hwMap = null;
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        op_mode = opMode;
        hwMap = ahwMap;
        m0 = hwMap.get(DcMotor.class, "intake_m0");
        m1 = hwMap.get(DcMotor.class, "intake_m1");
        dropper1 = hwMap.get(Servo.class, "dropper1");
        dropper2 = hwMap.get(Servo.class, "dropper2");
    }
    public void deploy(){
        dropper1.setPosition(0.3);
        dropper2.setPosition(0);
    }
    public void retract(){
        dropper1.setPosition(0);
        dropper2.setPosition(0.3);
    }
    public void turnOn(){
        m0.setPower(0.8);
        m1.setPower(0.8);
    }

    public void turnOff() {
        m0.setPower(0);
        m1.setPower(0);
    }
}
