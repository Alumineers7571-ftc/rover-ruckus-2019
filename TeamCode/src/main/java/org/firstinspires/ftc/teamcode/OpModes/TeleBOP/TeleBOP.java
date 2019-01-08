package org.firstinspires.ftc.teamcode.OpModes.TeleBOP;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp
public class TeleBOP extends LinearOpMode {

    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry, false);

        while(!isStarted() && !isStopRequested()){

            telemetry.addData("opmode:", "waiting....");
            telemetry.update();
        }

        while(opModeIsActive() && !isStopRequested()){


            robot.drive.setVelocity(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

            robot.mineralSystem.controlSystem(gamepad2, telemetry);
            robot.hanger.controlHanger(gamepad2);

            idle();
        }


    }
}
