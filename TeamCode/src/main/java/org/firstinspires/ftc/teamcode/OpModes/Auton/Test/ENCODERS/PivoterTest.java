package org.firstinspires.ftc.teamcode.OpModes.Auton.Test.ENCODERS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.PIDController;

@TeleOp
public class PivoterTest extends LinearOpMode {

    PIDController pid;

    DcMotorEx pivoter;

    double correction, power = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {

        pivoter = hardwareMap.get(DcMotorEx.class, "pivoter");

        pid = new PIDController(.007, 0, 0);

        pid.setSetpoint(0);
        pid.setOutputRange(0, power);
        pid.setInputRange(-1120, 1120);
        pid.enable();

        pivoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_bumper){

                pid.disable();
                pid.setSetpoint(1120/2);
                pid.enable();

            } else if(gamepad1.left_bumper) {

                pid.disable();
                pid.setSetpoint(0.0);
                pid.enable();

            } else {

                correction = pid.performPID(pivoter.getCurrentPosition());
                if (correction < 0.1 && correction > -0.1){
                    correction = 0;
                }
                pivoter.setPower(correction);

            }

            telemetry.addData("error", pid.getError());
            telemetry.addData("correction", correction);
            telemetry.update();

        }

    }
}

