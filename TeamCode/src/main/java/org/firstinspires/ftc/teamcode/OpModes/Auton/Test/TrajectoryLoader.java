package org.firstinspires.ftc.teamcode.OpModes.Auton.Test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Hardware.drive.SampleMecanumDriveREV;

import java.io.File;

@Autonomous
public class TrajectoryLoader extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        File file = new File("TeamCode/src.main/assets/trajectory");

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(60)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            Pose2d currentPose = drive.getPoseEstimate();

            drive.update();


        }
    }
}