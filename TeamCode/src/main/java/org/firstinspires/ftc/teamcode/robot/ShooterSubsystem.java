package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSubsystem {
    public DcMotorEx shooter1 = null;   // shooter motor
    public DcMotorEx shooter2 = null;   // shooter motor

    public Servo pusher = null;
    public Servo flip = null;
    int count = 0;
    HardwareMap hwMap = null;
    double power = 1950;
    public ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        shooter1 = hwMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hwMap.get(DcMotorEx.class, "shooter2");

        pusher = hwMap.get(Servo.class, "pusher");
        flip = hwMap.get(Servo.class, "flip");
        //shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        neutral();
    }

    public void turnOn() {
        //shooter1.setPower(power/2000.0);
        shooter1.setVelocity(power);
        shooter2.setVelocity(power);
    }

    public void turnOff() {
        //shooter1.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }

    public void trigger() {
        pusher.setPosition(0.31);
    }
    public void neutral() {
        pusher.setPosition(0.465);
    }

    public void shootPos(){flip.setPosition(0.69);}
    public void storePos(){flip.setPosition(0.98);}
    public boolean shoot() {
        double mil = runtime.milliseconds();
        if (count == 3) {
            count=0;
            return false;
        }
        if (mil > 0 && mil < 200)trigger();
        if (mil > 200 && mil < 400){
            neutral();
        }
        if (mil > 600) {
            restartTime();
            count++;
        }
        return true;
    }


    public void restartTime() {
        runtime.reset();
        runtime.startTime();
    }
    public void setPower(double p){
         power = p;
    }
    public double getPower(){
        return power;
    }
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
