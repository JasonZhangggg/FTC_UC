package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.ArmSubsystem;
import org.firstinspires.ftc.teamcode.robot.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.robot.MecDrive;
import org.firstinspires.ftc.teamcode.robot.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.vision.UGRectDetector;

@Autonomous(name = "redSide", group = "UltimateGoal")
//@Disabled
public class redSide extends LinearOpMode {
    //IntakeSubsystem intake = new IntakeSubsystem();
    ShooterSubsystem shooter = new ShooterSubsystem();
    ArmSubsystem arm = new ArmSubsystem();
    UGRectDetector vision = new UGRectDetector();
//    ElapsedTime runtime = new ElapsedTime();
//    String ringNum = "";
    @Override
    public void runOpMode() {
        //intake.init(hardwareMap, this, "teleop");
        MecDrive drive = new MecDrive(hardwareMap);
        shooter.init(hardwareMap);
        arm.init(hardwareMap);
        vision.init(hardwareMap, "Webcam");
        waitForStart();

        if (isStopRequested()) return;

        String ringNum = vision.getStack().toString();
        telemetry.addData("ringNum", ringNum);
        telemetry.update();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-50, -15), Math.toRadians(180))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-60, 10))
                .build();
        Trajectory traj3;
        if(ringNum.equals("ZERO")) {
            traj3 = drive.trajectoryBuilder(traj2.end())
                    .strafeTo(new Vector2d(-60, 26))
                    .build();
        }
        else if(ringNum.equals("ONE")){
             traj3 = drive.trajectoryBuilder(traj2.end())
                    .strafeTo(new Vector2d(-82, 5))
                    .build();
        }
        else{
            traj3 = drive.trajectoryBuilder(traj2.end())
                    .strafeTo(new Vector2d(-105, 26))
                    .build();
        }
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(5)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(-40, 18, Math.toRadians(180)))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .back(20)
                .build();
        Trajectory traj7;
        if(ringNum.equals("ZERO")) {
            traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineToLinearHeading(new Pose2d(-60, 18,  Math.toRadians(-20)))
                    .build();
        }
        else if(ringNum.equals("ONE")) {
            traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineToLinearHeading(new Pose2d(-82, 5, Math.toRadians(-20)))
                    .build();
        }
        else{
            traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineToLinearHeading(new Pose2d(-105, 16, Math.toRadians(-20)))
                    .build();
        }
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .forward(5)
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToLinearHeading(new Pose2d(-75, -5, Math.toRadians(180)))
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        /*
        shooter.turnOn();
        sleep(200);
        shoot();
        shoot();
        shoot();
        shoot();
        shooter.turnOff();*/
        //sleep(500);
        drive.followTrajectory(traj3);
        arm.move(170);
        sleep(1000);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        arm.move(0);
        sleep(1000);
    }
    /*
    public void shoot(){
        shooter.trigger();
        sleep(200);
        shooter.neutral();
        sleep(200);
    }*/
}