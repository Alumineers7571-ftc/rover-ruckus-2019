package org.firstinspires.ftc.teamcode.OpModes.Auton.Test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.QuinticSplineSegment;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.PathTrajectorySegment;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.drive.SampleMecanumDriveREV;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class RoadRunnerTesting extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        DriveConstraints constraints = DriveConstants.BASE_CONSTRAINTS;

        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);

        Path path = new Path(new QuinticSplineSegment(
                new QuinticSplineSegment.Waypoint(10, 10, 10, 0), // start position and derivatives
                new QuinticSplineSegment.Waypoint(20, 20, 0, 10) // end position and derivatives
        ), new ConstantInterpolator(Math.toRadians(45)));

        Path constantHeadingSpline = new Path(new QuinticSplineSegment(
                new QuinticSplineSegment.Waypoint(0, 0, 0, 0), // start position and derivatives
                new QuinticSplineSegment.Waypoint(0, 101.6, 0, 0) // end position and derivatives
        ));

        Path constantHeadingSpline2 = new Path(new QuinticSplineSegment(
                new QuinticSplineSegment.Waypoint(0, 0, 0, 0), // start position and derivatives
                new QuinticSplineSegment.Waypoint(36.83*2, 101.6, 0, 0) // end position and derivatives
        ));

        Trajectory trajectoryBasic = new Trajectory(Arrays.asList(
                new PathTrajectorySegment(constantHeadingSpline2, constraints)
        ));

        Trajectory constantHeadingTrajectory = new Trajectory(Arrays.asList(
                new PathTrajectorySegment(constantHeadingSpline, constraints)
        ));


        while(!isStarted() && !isStopRequested()){

            telemetry.addLine("waiting for you :)");
            telemetry.update();
        }

        /*drive.followTrajectory(trajectoryBasic);
        while(!isStopRequested() && drive.isFollowingTrajectory()){
            drive.update();
        }*/

        drive.followTrajectory(constantHeadingTrajectory);
        while(!isStopRequested() && drive.isFollowingTrajectory()){
            drive.update();
        }



    }
}
