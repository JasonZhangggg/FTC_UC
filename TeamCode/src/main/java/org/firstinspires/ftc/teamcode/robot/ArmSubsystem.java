package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem {
    public Servo wobbleArm = null;
    public Servo wobbleClaw = null;   // arm motor
    final int UP_POS = 0;
    final int DOWN_POS = -130;
    // TODO: need 2 touch sensors to control the degree of rotation

    HardwareMap hwMap = null;
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        wobbleArm = hwMap.get(Servo.class, "wobbleArm");
        wobbleClaw = hwMap.get(Servo.class, "wobbleClaw");
        rotateBackward(true);
        close();
    }

    public void rotateForward(){
        wobbleArm.setPosition(0);
    }
    public void rotateBackward(boolean high){
        if(high)wobbleArm.setPosition(1);
        else wobbleArm.setPosition(0.4);
    }
    public void close(){
        wobbleClaw.setPosition(1);
    }
    public void open(){
        wobbleClaw.setPosition(0.7);
    }
}
