package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSubsystem {
    public DcMotor arm = null;   // arm motor
    final int UP_POS = 0;
    final int DOWN_POS = -130;
    // TODO: need 2 touch sensors to control the degree of rotation

    HardwareMap hwMap = null;
    ElapsedTime runtime = new ElapsedTime();
    int pos;
    public void init(HardwareMap ahwMap) {
        pos = 0;
        hwMap = ahwMap;
        arm = hwMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void rotateForward(){
        move(UP_POS);
    }

    public void rotateBackward(){
        move(DOWN_POS);
    }
    public void setPos(int newPos){
        pos = newPos;
    }
    public int getPos(){
        return pos;
    }
    public void move(int newPos){
        arm.setTargetPosition(newPos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.6);
    }
}
