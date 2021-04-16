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

@Autonomous(name = "redSide2", group = "UltimateGoal")
//@Disabled
public class redSide2 extends LinearOpMode {
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
        String ringNum = vision.getStack().toString();
        telemetry.addData("ringNum", ringNum);
        telemetry.update();

        Trajectory traj_to_shoot_pos = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(50, 0))
                .build();
        Trajectory traj_back_from_ringstack = drive.trajectoryBuilder(traj_to_shoot_pos.end())
                .back(5)
                .build();

        Trajectory traj_to_ringstack1 = drive.trajectoryBuilder(traj_back_from_ringstack.end())
                .forward(3)
                .build();
        Trajectory traj_to_ringstack2 = drive.trajectoryBuilder(traj_to_ringstack1.end())
                .forward(3)
                .build();
        Trajectory traj_to_ringstack3 = drive.trajectoryBuilder(traj_to_ringstack2.end())
                .forward(3)
                .build();

        //___________________________________________________________________________________________
        arm.rotateBackward(false);

        //drive to shoot position w/ intake on reversely
        intake.setReverse();
        intake.turnOnSlow();
        drive.followTrajectory(traj_to_shoot_pos);
        intake.unsetReverse();
        intake.turnOff();

        shooter.setPower(1850);
        shooter.turnOn();
        shooter.shootPos();

        // first 3 rings
        sleep(1750);
        shoot();
        shoot();
        shoot();
        shooter.turnOff();

        drive.followTrajectory(traj_back_from_ringstack);


        // intake rings
        intake.deploy();
        shooter.storePos();
        intake.turnOn();
        drive.followTrajectory(traj_to_ringstack1);
        sleep(2000);
        drive.followTrajectory(traj_to_ringstack2);
        sleep(2000);
        drive.followTrajectory(traj_to_ringstack3);
        sleep(2000);

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