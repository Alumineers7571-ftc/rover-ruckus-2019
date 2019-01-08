package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Hardware.drive.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.Hardware.roverruckus.Hanger;
import org.firstinspires.ftc.teamcode.Hardware.roverruckus.MineralSystem;
import org.firstinspires.ftc.teamcode.Hardware.roverruckus.TeamMarkerer;

public class Robot{

    public SampleMecanumDriveBase drive;
    public MineralSystem mineralSystem = new MineralSystem();
    public Hanger hanger = new Hanger();
    public TeamMarkerer tm = new TeamMarkerer();


    public Robot() {

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto){
        this.initialize(hardwareMap, telemetry, isAuto);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry, boolean isAuto) {

        drive = new SampleMecanumDriveREV(hardwareMap);
        mineralSystem.init(hardwareMap, telemetry);
        hanger.init(hardwareMap, telemetry);
        tm.init(hardwareMap, telemetry);


    }

}
