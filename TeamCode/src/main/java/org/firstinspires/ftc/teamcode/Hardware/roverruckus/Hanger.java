package org.firstinspires.ftc.teamcode.Hardware.roverruckus;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Hanger{

    DcMotorEx hanger;

    DistanceSensor distanceSensor;

    DigitalChannel digitalTouch;  // Hardware Device Object

    boolean isAtGround = false;

    final double SENSOR_IN_FROM_GROUND = 2.7;

    public Hanger(){
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        hanger = hardwareMap.get(DcMotorEx.class,"hanger");

        hanger.setDirection(DcMotorSimple.Direction.REVERSE); //its geared

        distanceSensor = hardwareMap.get(DistanceSensor.class, "range");

        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        double startingDistanceIn = distanceSensor.getDistance(DistanceUnit.INCH);
        boolean scored = false;

        isAtGround = false;

        if(startingDistanceIn < 4){
            scored = false;
        } else {
            scored = true;
        }

        telemetry.addData("4+ ?: ", scored);
        telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

    }

    public double getDistanceToGround(DistanceUnit unit){

        return distanceSensor.getDistance(unit);
    }

    public void controlHanger(Gamepad gamepad){

        hanger.setPower(gamepad.right_trigger);

        if(gamepad.left_trigger > 0.05 && !isTouched()){
            moveToLowerLimit();
        }

    }

    public void controlHanger(double power){

        hanger.setPower(power);

    }

    public void moveToGround(){

        if(distanceSensor.getDistance(DistanceUnit.INCH) < (SENSOR_IN_FROM_GROUND)){

            hanger.setPower(0);
            isAtGround = true;

        } else {

            hanger.setPower(1);
            isAtGround = false;

        }


    }

    public boolean moveToLowerLimit(){

        if(isTouched()){

            hanger.setPower(0);
            return true;

        } else {

            hanger.setPower(-1);
            return false;

        }
    }

    public boolean isTouched(){
        return !digitalTouch.getState();
    }

    public boolean isAtGround(){

        return isAtGround;
    }


}
