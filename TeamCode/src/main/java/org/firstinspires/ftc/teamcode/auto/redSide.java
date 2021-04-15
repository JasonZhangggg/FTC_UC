package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(33, 7))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(60, 5))
                .build();
        Trajectory traj3;
        if (ringNum.equals("ZERO")) {
            traj3 = drive.trajectoryBuilder(traj2.end())
                    .strafeTo(new Vector2d(60, -27))
                    .build();
        } else if (ringNum.equals("ONE")) {
            traj3 = drive.trajectoryBuilder(traj2.end())
                    .strafeTo(new Vector2d(82, -3))
                    .build();
        } else {
            traj3 = drive.trajectoryBuilder(traj2.end())
                    .strafeTo(new Vector2d(100, -27))
                    .build();
        }
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(5)
                .build();
        Trajectory traj5;
        if(ringNum.equals("FOUR")) {
            traj5 = drive.trajectoryBuilder(traj4.end())
                    .lineToLinearHeading(new Pose2d(28, -21.5, Math.toRadians(180)))
                    .build();
        }
        else{
            traj5 = drive.trajectoryBuilder(traj4.end())
                    .lineToLinearHeading(new Pose2d(28, -23, Math.toRadians(180)))
                    .build();
        }
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(8)
                .build();
        Trajectory traj7;
        if (ringNum.equals("ZERO")) {
            traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineToLinearHeading(new Pose2d(60, -21, 0))
                    .build();
        } else if (ringNum.equals("ONE")) {
            traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineToLinearHeading(new Pose2d(82, 3, 0))
                    .build();
        } else {
            traj7 = drive.trajectoryBuilder(traj6.end())
                    .lineToLinearHeading(new Pose2d(100, -21, 0))
                    .build();
        }
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .back(5)
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToLinearHeading(new Pose2d(75, 5, 0))
                .build();




        //___________________________________________________________________________________________
        arm.rotateBackward(false);
        shooter.shootPos();
        //drive to shooting position
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        //turn shooter on and shoot 3 times
        shooter.turnOn();
        sleep(1750);
        shoot();
        shoot();
        shoot();
        shooter.turnOff();
        //drive to first wobble goal location
        drive.followTrajectory(traj3);
        drop();
        //move backwards away from the wobble goal
        drive.followTrajectory(traj4);
        //drive to the second wobble goal
        drive.followTrajectory(traj5);
        //go forward into the second wobble goal
        drive.followTrajectory(traj6);
        //pick up the wobble goal
        pick();
        //move to the second wobble goal
        drive.followTrajectory(traj7);
        //drop the second wobble goal
        drop();
        //back away from the wobble goal
        drive.followTrajectory(traj8);
        //move to the white line
        drive.followTrajectory(traj9);
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