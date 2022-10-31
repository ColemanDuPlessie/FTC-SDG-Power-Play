package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.GravityPIDFController;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class SlidesSubsystem extends Subsystem implements PositionControlled {

    public DcMotor motor; // TODO

    private final PIDController PIDF;

    public static int minPosition = -60; // We don't actually want to go all the way down.
    public static int maxPosition = -3700;

    public static double kP = 0.005;
    public static double kI = 0.0000;
    public static double kD = 0.0001;
    public static double kG = 0.1;
    public static double powerMultThrottle = 0.75;

    private int targetPosition;

    private final int startPosition;

    public SlidesSubsystem(ElapsedTime aTimer, HardwareMap ahwMap) {
        super(aTimer, ahwMap);
        motor = ahwMap.get(DcMotor.class, "SlidesMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = motor.getCurrentPosition();
        targetPosition = 0;
        PIDF = new GravityPIDFController(kP, kI, kD, aTimer, kG);
    }

    public SlidesSubsystem(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        super(aTimer, ahwMap);
        motor = ahwMap.get(DcMotor.class, "SlidesMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (isTeleop) {
            Integer position = AutoToTeleopContainer.getInstance().getSlidesPosition();
            if (position == null) {
                startPosition = motor.getCurrentPosition();
            } else { startPosition = position;}
        } else {
            startPosition = motor.getCurrentPosition();
            AutoToTeleopContainer.getInstance().setSlidesPosition(startPosition);
        }
        targetPosition = 0;
        PIDF = new GravityPIDFController(kP, kI, kD, aTimer, kG);
    }

    public double getPosition() {return ((double)(motor.getCurrentPosition()-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public void setTargetPosition(double target) {
        targetPosition = (int)(target * (maxPosition-minPosition) + minPosition);
    }

    @Override
    public void update() {
        motor.setPower(Math.min(powerMultThrottle, Math.max(PIDF.update(motor.getCurrentPosition()-startPosition, targetPosition) * powerMultThrottle, -powerMultThrottle)));
    }

}
