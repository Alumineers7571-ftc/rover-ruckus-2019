package org.firstinspires.ftc.teamcode.OpModes.Auton.Main;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.QuinticSplineSegment;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Hardware.drive.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryLoader;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.ENUMS;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous
public class Crater extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    PIDController pidRotate;

    GoldAlignDetector detector;

    ENUMS.GoldPosition goldPos = ENUMS.GoldPosition.UNKNOWN;

    ENUMS.AutoStates robo = ENUMS.AutoStates.START;

    Robot robot = new Robot();

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .7, correction;

    double distance;

    double xPos = 0;

    final double DISTANCE_FROM_TM_TO_CRATER = 100; //inches

    int distToDepot = 120;
    int distToCrater = -190;

    //SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry, true);

        pidRotate = new PIDController(0.01, 0, 0);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();

        vuforiaParameters.vuforiaLicenseKey = "AVLeWaD/////AAABmaCnSNdyYU2IrXuTK6ZXofgXezmDHUWmdN65PXvQbYrt89npFHJWC5yBgXMWvuHQJfgkJg3f5/U4ZwnCdlNiuIuW+eOi7Nhkob8UlCzcciD8tieNsPD+sYd4O/E/v2WFIDdgx9o4rcAcAM8gHu48dGUrlRQsfruhANKUk1RE6h0/xFJMKimqs0/7U3z6aoEoRnPajrSBifkIVpePsARuwDhlaszzsbc2vijwg+dkIVcEMckjPAk5YoPi7i30N8/vrmhrvdNaVUUzCFPsGo42ad3U/XmZwb7xKAEpU03goBoRQIjBgJegGWNBkKw7So8ilLSU9uHAYtZoiRyFnUy8YcVyU7YkTjA7ixzrDpV5Zah/";
        vuforiaParameters.fillCameraMonitorViewParent = true;

        vuforiaParameters.cameraName = webcamName;

        vuforia = new Dogeforia(vuforiaParameters);
        vuforia.enableConvertFrameToBitmap();



        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        //For convenience, gather together all the trackable objects in one easily-iterable collection

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        //--------------------------------------------------------------------------------------------------
        // CHANGE these when you can
        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line
        //--------------------------------------------------------------------------------------------------

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();

        detector.downscale = 0.5; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

        /*Trajectory landerToSample = robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(20, 55, 0))
                .build();
*/



        AssetsTrajectoryLoader loader = new AssetsTrajectoryLoader();

        Trajectory leftGoldTrajectory = null, rightGold = null, middleGold = null;

        leftGoldTrajectory = robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(50, -25, 0))
                .back(20)
                .build();

        rightGold = robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(50, 25, 0))
                .back(20)
                .build();

        middleGold = robot.drive.trajectoryBuilder()
                .forward(50)
                .back(20)
                .build();

        Trajectory sampleTrajectory = robot.drive.trajectoryBuilder()
                .forward(50)
                .back(20)
                .build();

        Path sampleToDepot = new Path(new QuinticSplineSegment(
                new QuinticSplineSegment.Waypoint(0, 0, 0, 0), // start position and derivatives
                new QuinticSplineSegment.Waypoint(36.83*2, 101.6, 0, 0) // end position and derivatives
        ));

        while(!isStarted() && !isStopRequested()){

            xPos = detector.getXPosition();

            if(detector.isFound()) {
                if (xPos > -10 && xPos < 180) {
                    goldPos = ENUMS.GoldPosition.LEFT;
                    sampleTrajectory = leftGoldTrajectory;
                } else if (xPos >= 180 && xPos < 420) {
                    goldPos = ENUMS.GoldPosition.CENTER;
                    sampleTrajectory = middleGold;
                } else if (xPos >= 420 && xPos < 610) {
                    goldPos = ENUMS.GoldPosition.RIGHT;
                    sampleTrajectory = rightGold;
                }
            } else {
                goldPos = ENUMS.GoldPosition.UNKNOWN;
            }

            telemetry.addLine("x pos: " + xPos);
            telemetry.addLine("pos: " + goldPos);
            telemetry.addLine("distance from ground: " + robot.hanger.getDistanceToGround(DistanceUnit.INCH));
            telemetry.addData("opmode:", "waiting for start....");
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {

            switch (robo) {

                case START: {

                    telemetry.addLine("GOLD LOCATION: " + goldPos);
                    telemetry.update();

                    robo = ENUMS.AutoStates.DROPDOWN;
                    break;
                }

                case DROPDOWN: {

                    while(opModeIsActive() && !robot.hanger.isAtGround()){
                        robot.hanger.moveToGround();
                    }
                    robot.hanger.controlHanger(-.5);
                    sleep(300);
                    robot.hanger.controlHanger(0);
                    rotate(-178);

                    robo = ENUMS.AutoStates.MOVETOSAMPLE;
                    break;
                }

                case FINDGOLD: {



                    robo = ENUMS.AutoStates.MOVETOSAMPLE;
                    break;
                }

                case MOVETOSAMPLE: {

                    robot.drive.setDirectionReverse(DcMotorSimple.Direction.FORWARD);

                    robot.drive.followTrajectory(sampleTrajectory);

                    while(!isStopRequested() && robot.drive.isFollowingTrajectory()){
                        robot.drive.update();
                    }

                    robot.drive.setDirectionForwards(DcMotorSimple.Direction.FORWARD);

                    rotate(90);



                    if(goldPos == ENUMS.GoldPosition.LEFT){
                        distance = 72; //???
                    } else if(goldPos == ENUMS.GoldPosition.CENTER) {
                        distance = 72-14.5; //see above
                    } else {
                        distance = 72 - (14.5 * 2);
                    }

                    robo = ENUMS.AutoStates.FINDWALLFORDEPOT;
                    break;
                }
                    
                case FINDWALLFORDEPOT: {

                    robot.drive.encoderDrive(0.3, distance, distance, opModeIsActive());

                    telemetry.update();

                    rotate(45);

                    telemetry.update();

                    robot.drive.encoderDrive(0.3, distToDepot, distToDepot, opModeIsActive());

                    robo = ENUMS.AutoStates.DROPTM;
                    break;
                }
                    
                case DROPTM: {
                 
                    robot.tm.setTMDown();
                    sleep(750);

                    robot.drive.encoderDrive(0.6, -distToCrater, -distToCrater, opModeIsActive());
                    
                    robo = ENUMS.AutoStates.END;
                    break;
                }
                    
                case END:{
                    
                     
                }
            }

            //telemetry.log().clear();
            //telemetry.log().add("x of gold: " + detector.getXPosition());
            telemetry.addLine("angle: " + robot.drive.getExternalHeading());
            telemetry.addLine("state: " + robo);
            telemetry.update();

        }
    }

    private void resetAngle() {
        lastAngles = robot.drive.getAOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void rotate(int degrees) {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 180);
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // rb.drive.getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.drive.getExternalHeading() == 0) {
                robot.drive.setMotorPowers(power, power, -power, -power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(robot.drive.getExternalHeading()); // power will be - on right turn.
                robot.drive.setMotorPowers(-power, -power, power, power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(robot.drive.getExternalHeading()); // power will be + on left turn.
                robot.drive.setMotorPowers(-power, -power, power, power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.drive.setThrottle(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

}
