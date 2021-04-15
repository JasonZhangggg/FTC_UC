
package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.MecDrive;

import org.firstinspires.ftc.teamcode.vision.UGRectDetector;

@TeleOp(name = "TeleOp", group = "UltimateGoal")
//@Disabled
public class UGTeleop extends LinearOpMode {
    IntakeSubsystem intake = new IntakeSubsystem();
    ShooterSubsystem shooter = new ShooterSubsystem();
    ArmSubsystem arm = new ArmSubsystem();
    double speed = 1;
    ElapsedTime flipTime = new ElapsedTime();
    boolean buttonPrev[] = {false, false, false, false, false, false, false, false, false}; //bumper_left, bumper_right, gamepad2_y, dpad_up, b
    // TODO: fix those variable names
    //forward speed
    double ch3 = 0;
    //turn speed
    double ch1;
    //strafe speed
    double ch4 = 0;
    //are we shooting rings
    boolean tripleShoot = false;
    //manual or auto power adjustment
    boolean autoPower = true;
    boolean shooting = false;
    boolean wobbleOpen = false;
    @Override
    public void runOpMode() {
        MecDrive drive = new MecDrive(hardwareMap);
        intake.init(hardwareMap, this);
        shooter.init(hardwareMap);
        arm.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-2, -12, 0))
                .build();
        intake.deploy();
        waitForStart();

        while (opModeIsActive()) {
            // ### Control speed
            if (gamepad1.b) {
                speed = 0.5;
            } else if (gamepad1.a) {
                speed = 1;
            } else if (gamepad1.dpad_down) {
                speed = 0.3;
            }

            // ### both off
            if (gamepad2.a) {
                intake.turnOff();
                shooter.turnOff();
                shooting = false;
            }
            // ### intake on shooter off
            else if (gamepad2.x) {
                shooter.turnOff();
                intake.turnOn();
                shooter.storePos();
                arm.rotateBackward(true);
                shooting = false;
            }
            // ### intake off shooter on
            else if(buttonClick(gamepad2.b, 6)){
                shooting = true;
                flipTime.reset();
                flipTime.startTime();
                intake.turnOff();
                shooter.shootPos();
                arm.rotateForward();
            }
            if(flipTime.milliseconds()>350 && !tripleShoot && shooting){
                shooter.turnOn();
            }

            //triple shoot
            if (buttonClick(gamepad2.y, 2)) {
                tripleShoot = true;
                shooter.restartTime();
            }
            if (tripleShoot) {
                tripleShoot = shooter.shoot();
            } else {
                if (gamepad2.right_trigger > 0.6) {
                    shooter.trigger();
                } else {
                    shooter.neutral();
                }
            }
            if (buttonClick(gamepad2.dpad_left, 3)) {
                shooter.setPower(shooter.getPower() - 100);
            }
            if (buttonClick(gamepad2.dpad_right, 7)) {
                shooter.setPower(shooter.getPower() + 100);
            }

            if (buttonClick(gamepad2.right_bumper, 1)) {
                shooter.setPower(shooter.getPower() + 25);
            }
            if (buttonClick(gamepad2.left_bumper, 0)) {
                shooter.setPower(shooter.getPower() - 25);
            }
            if (buttonClick(gamepad1.x, 4)) {
                drive.followTrajectory(traj);
            }
            if(gamepad2.left_stick_y<-0.6){
                arm.rotateBackward(true);
            }
            else if(gamepad2.left_stick_y>0.8){
                arm.rotateForward();
            }
            else if(gamepad2.left_stick_y>0.4 && gamepad2.left_stick_y<0.8){
                arm.rotateBackward(false);
            }
            if(buttonClick(gamepad2.left_trigger > 0.6, 8)){
                if(wobbleOpen){
                    arm.close();
                }
                else{
                    arm.open();
                }
                wobbleOpen = !wobbleOpen;
            }
            telemetry.addData("Power", shooter.getPower());
            telemetry.addData("Voltage", shooter.getBatteryVoltage());
            telemetry.update();

            // ### Drive the robot
            ch4 = gamepad1.right_bumper ? -0.9 : gamepad1.left_bumper ? 0.9 : 0;
            /*
            ch3 = gamepad1.left_stick_y > 0.4 ? 0.9 : gamepad1.left_stick_y < -0.4 ? -0.9 : 0;
            ch4 = gamepad1.right_bumper ? -0.9 : gamepad1.left_bumper ? 0.9 : 0;
            ch1 = gamepad1.right_stick_x > 0.4 ? -0.6 : gamepad1.right_stick_x < -0.4 ? 0.6 : 0;
            robot.leftDriveFront.setPower((ch3 + ch1 + ch4) * speed);
            robot.leftDriveBack.setPower((ch3 + ch1 - ch4) * speed);
            robot.rightDriveFront.setPower((ch3 - ch1 - ch4) * speed);
            robot.rightDriveBack.setPower((ch3 - ch1 + ch4) * speed);*/
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*speed,
                            ch4*speed,
                            -gamepad1.right_stick_x*speed
                    )
            );

            drive.update();

        }
    }

    public boolean buttonClick(boolean button, int loc) {
        boolean returnVal = button && !buttonPrev[loc];
        buttonPrev[loc] = button;
        return returnVal;
    }
}