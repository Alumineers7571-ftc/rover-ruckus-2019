package org.firstinspires.ftc.teamcode.Hardware.roverruckus;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class MineralSystem{

    DcMotorEx extendo, pivoter;

    PIDController pid;

    boolean pidOn = false;

    double correction, power = 0.7;


    CRServo intakeLeft, intakeRight;

    Servo gate;

    private static final double SERVO_LITTLE_POS = .25;
    private static final double SERVO_UP_POS = 1;
    private static final double SERVO_DOWN_POS = 0;

    private boolean servoSet = false;

    private boolean isIntakeRunning = false;
    private boolean isHangerRunning = false;
    private boolean isExtendoRunning = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        pivoter = hardwareMap.get(DcMotorEx.class, "pivoter");

        intakeLeft = hardwareMap.crservo.get("intakeL");
        intakeRight = hardwareMap.crservo.get("intakeR");

        gate = hardwareMap.servo.get("gate");

        pid = new PIDController(.005, 0, 0);

        pid.setSetpoint(0);
        pid.setOutputRange(0, power);
        pid.setInputRange(-1120*2, 1120*2);
        pid.setTolerance(1);
        pid.enable();

        pivoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine(getClass().toString() + " setup");
        telemetry.update();
    }

    public void controlSystem(Gamepad gamepad, Telemetry telemetry){

        extendo.setPower(gamepad.right_stick_y);

        if(gamepad.left_bumper){
            runIntake(1);
        } else if(gamepad.right_bumper) {
            runIntake(-1);
        } else {
            runIntake(0);
        }

        if(gamepad.b){
            gate.setPosition(1);
        } else {
            gate.setPosition(0);
        }

        if(gamepad.left_stick_button){
            pidOn = !pidOn;
        }

        if(pidOn){

            if (gamepad.dpad_right) {

                changePivotSetpoint(0);

            } else if (gamepad.dpad_left) {

                changePivotSetpoint(-1680 / 4 * 3);

            } else if (gamepad.dpad_up) {

                changePivotSetpoint(-1580);

            } else {

                correction = pid.performPID(pivoter.getCurrentPosition());
                /*if (correction < 0.08 && correction > -0.08) {
                    correction = 0;
                }*/
                pivoter.setPower(correction);



            }
        } else {
            pivoter.setPower(gamepad.left_stick_y);
        }

        telemetry.addData("correction", correction);
        telemetry.addData("ticks", pivoter.getCurrentPosition());
        telemetry.update();
    }

    public void runIntake(double power){

        if (power != 0){
            isIntakeRunning = true;
        } else {
            isIntakeRunning = false;
        }

        //positive power is intaking
        //negative power is outtaking

        intakeLeft.setPower(power);
        intakeRight.setPower(-power);

    }

    /*
    changes the position that the pivoter will hold
     */
    public void changePivotSetpoint(double setpoint){

        pid.disable();
        pid.setSetpoint(setpoint);
        pid.enable();

    }

}
