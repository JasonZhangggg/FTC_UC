package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.ArmSubsystem;
import org.firstinspires.ftc.teamcode.robot.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.MecDrive;
import org.firstinspires.ftc.teamcode.robot.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.vision.UGRectDetector;

@Autonomous(name = "powerShot", group = "UltimateGoal")
//@Disabled
public class powerShot extends LinearOpMode {
    IntakeSubsystem intake = new IntakeSubsystem();
    ShooterSubsystem shooter = new ShooterSubsystem();
    ArmSubsystem arm = new ArmSubsystem();
    UGRectDetector vision = new UGRectDetector();

    @Override
    public void runOpMode() {
        intake.init(hardwareMap, this);
        MecDrive drive = new MecDrive(hardwareMap);
        shooter.init(hardwareMap);
        arm.init(hardwareMap);
        vision.init(hardwareMap, "Webcam");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        if (isStopRequested()) return;

        //___________________________________________________________________________________________
        arm.rotateBackward(false);

       // drive.turn(Math.toRadians(-8));

        shooter.setPower(1750);
        shooter.turnOn();
        shooter.shootPos();

        // first 3 rings
        sleep(1750);

        shoot();
        drive.turn(Math.toRadians(-6));
        shoot();
        drive.turn(Math.toRadians(12));
        shoot();
        shooter.turnOff();


    }

    public void shoot() {
        shooter.trigger();
        sleep(200);
        shooter.neutral();
        sleep(200);
    }

    public void drop() {
        arm.rotateForward();
        sleep(500);
        arm.open();
        sleep(250);
    }
    public void pick(){
        arm.close();
        sleep(500);
        arm.rotateBackward(false);
    }
}