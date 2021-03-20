package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.UGRectDetector;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class MySplineTest extends LinearOpMode {
    ShooterSubsystem shooter = new ShooterSubsystem();
    @Override
    public void runOpMode() throws InterruptedException {
        shooter.init(hardwareMap);
        MecDrive drive = new MecDrive(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-50, -15), Math.toRadians(180))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-60, 10))
                //.strafeLeft(10)
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);

        shoot();
        shoot();
        shoot();
        shoot();
        /*
        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );*/
    }
    public void shoot(){
        shooter.trigger();
        sleep(200);
        shooter.neutral();
        sleep(200);
    }
}
