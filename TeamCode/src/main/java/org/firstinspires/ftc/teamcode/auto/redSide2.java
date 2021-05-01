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

        Trajectory traj_to_ringstack0 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(25, -6))
                .build();
        Trajectory traj_to_shoot_pos = drive.trajectoryBuilder(traj_to_ringstack0.end())
                .forward(25)
                .build();

        Trajectory traj_to_ringstack1 = drive.trajectoryBuilder(traj_to_shoot_pos.end())
                .forward(2)
                .build();
        Trajectory traj_to_ringstack2 = drive.trajectoryBuilder(traj_to_ringstack1.end())
                .forward(2)
                .build();
        Trajectory traj_to_ringstack3 = drive.trajectoryBuilder(traj_to_ringstack2.end())
                .forward(3)
                .build();
      //  Trajectory traj_back_from_ringstack = drive.trajectoryBuilder(traj_to_ringstack3.end())
      //          .back(0.1)
      //          .build();

        // wobble goals
        Trajectory traj_drop_1st_wg = drive.trajectoryBuilder(traj_to_ringstack3.end())
                .strafeTo(new Vector2d(100, -25))
                .build();
        Trajectory traj_drop_1st_wg_back = drive.trajectoryBuilder(traj_drop_1st_wg.end())
                .back(5)
                .build();
        Trajectory traj_pick_2nd_wg = drive.trajectoryBuilder(traj_drop_1st_wg_back.end())
                .lineToLinearHeading(new Pose2d(28, -23, Math.toRadians(180)))
                .build();

        Trajectory traj_pick_2nd_wg_fwd = drive.trajectoryBuilder(traj_pick_2nd_wg.end())
                .forward(8)
                .build();
        Trajectory traj_drop_2nd_wg = drive.trajectoryBuilder(traj_pick_2nd_wg_fwd.end())
                .lineToLinearHeading(new Pose2d(100, -18, 0))
                .build();

        Trajectory traj_drop_2nd_wg_back = drive.trajectoryBuilder(traj_drop_2nd_wg.end())
                .back(25)
                .build();
        Trajectory traj_end = drive.trajectoryBuilder(traj_drop_2nd_wg_back.end())
                .lineToLinearHeading(new Pose2d(75, 5, 0))
                .build();

        //___________________________________________________________________________________________
        arm.rotateBackward(false);
        shooter.shootPos();
        intake.deploy();

        //drive to shoot position
        drive.followTrajectory(traj_to_ringstack0);
        shooter.turnOn();
        drive.followTrajectory(traj_to_shoot_pos);

        // first 3 rings
        drive.turn(Math.toRadians(9));
        // sleep(1000);
        shoot();
        shoot();
        shoot();
        shooter.turnOff();
        drive.turn(Math.toRadians(-9));

        // intake rings
        shooter.storePos();
        intake.turnOn();
        drive.followTrajectory(traj_to_ringstack1);
        //sleep(500);
        drive.followTrajectory(traj_to_ringstack2);
        //sleep(1000);
        drive.followTrajectory(traj_to_ringstack3);
        sleep(1500);
        intake.turnOff();

        // shoot (2nd)
        shooter.shootPos();
        sleep(300);
        shooter.setPower(2200);
        shooter.turnOn();
        // drive.followTrajectory(traj_back_from_ringstack);

        // first 3 rings
        drive.turn(Math.toRadians(9));
        //sleep(1000);
        shoot();
        shoot();
        shoot();
        shooter.turnOff();
        drive.turn(Math.toRadians(-9));

        // drop 1st whole goal
        drive.followTrajectory(traj_drop_1st_wg);
        drop();
        drive.followTrajectory(traj_drop_1st_wg_back);

        drive.followTrajectory(traj_pick_2nd_wg);
        drive.followTrajectory(traj_pick_2nd_wg_fwd);
        pick();
        drive.followTrajectory(traj_drop_2nd_wg);
        drop();
        drive.followTrajectory(traj_drop_2nd_wg_back);

        //drive.followTrajectory(traj_end);
    }

    public void shoot() {
        shooter.trigger();
        sleep(130);
        shooter.neutral();
        sleep(130);
    }

    public void drop() {
        arm.rotateForward();
        //sleep(10);
        arm.open();
        //sleep(50);
    }
    public void pick(){
        arm.close();
        sleep(100);
        arm.rotateBackward(false);
    }
}