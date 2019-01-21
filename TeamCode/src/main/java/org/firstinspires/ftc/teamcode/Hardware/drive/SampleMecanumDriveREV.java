package org.firstinspires.ftc.teamcode.Hardware.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware. If your hardware configuration
 * satisfies the requirements, SampleMecanumDriveREVOptimized is highly recommended.
 */
public class SampleMecanumDriveREV extends SampleMecanumDriveBase {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    private PIDController pidRotate;

    Trajectory trajectory;

    Orientation lastAngles = new Orientation();

    double correction, power = 0.7;

    double powerMul = 1;

    public SampleMecanumDriveREV(HardwareMap hardwareMap) {
        super();

        pidRotate = new PIDController(.005, 0, 0);

        pidRotate.setSetpoint(0);
        pidRotate.setOutputRange(0, power);
        pidRotate.setInputRange(-180, 180);
        pidRotate.enable();

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        //setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    public void setDirectionReverse(DcMotorSimple.Direction direction){
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setDirectionForwards(DcMotorSimple.Direction direction){
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    @Override
    public double getExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Orientation getAOrientation(AxesReference ref, AxesOrder order, AngleUnit unit) {
        return imu.getAngularOrientation(ref, order, unit);
    }

    @Override
    public void setThrottle(double power){
        setMotorPowers(power, power, power, power);
    }

    @Override
    public void controlSystem(Gamepad gamepad){

        setVelocity(new Pose2d(-gamepad.left_stick_y * powerMul, -gamepad.left_stick_x * powerMul, -gamepad.right_stick_x * powerMul));

        if(gamepad.right_bumper){
            powerMul = 0.4;
        } else if(gamepad.left_bumper){
            powerMul = 0.75;
        } else {
            powerMul = 1;
        }

    }


}
