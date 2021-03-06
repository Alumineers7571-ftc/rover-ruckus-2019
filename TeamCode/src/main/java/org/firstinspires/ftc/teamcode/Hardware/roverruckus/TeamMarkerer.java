package org.firstinspires.ftc.teamcode.Hardware.roverruckus;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamMarkerer{

    Servo teamMarkerer;

    final double POS_DOWN = 0.5;
    final double POS_UP = 1;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        teamMarkerer = hardwareMap.servo.get("tm");

        setTMUp();

        telemetry.addLine("tm good");
        telemetry.update();

    }

    public void setTMDown(){
        teamMarkerer.setPosition(POS_DOWN);
    }

    public void setTMUp(){
        teamMarkerer.setPosition(POS_UP);
    }

    public Servo getTeamMarkerer(){
        return teamMarkerer;
    }

}
