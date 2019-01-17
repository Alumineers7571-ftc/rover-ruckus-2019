package org.firstinspires.ftc.teamcode.Hardware.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecv.math.MathFTC;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Hardware.drive.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous
public class SplineTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .splineTo(new Pose2d(35, 40, 0))
                .back(16)
                .turn(-Math.PI/4)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        while (!isStopRequested() && drive.isFollowingTrajectory()) {


            drive.update();
        }
    }
}
