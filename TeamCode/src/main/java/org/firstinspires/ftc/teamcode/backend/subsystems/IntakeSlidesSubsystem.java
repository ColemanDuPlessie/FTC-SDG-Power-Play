package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class IntakeSlidesSubsystem extends SubsystemBase implements PositionControlled {

    public DcMotor motor;

    private PIDController PIDF;

    public static int minPosition = 0;
    public static int maxPosition = 1150;

    public static double kP = 0.008;
    public static double kI = 0.0000;
    public static double kD = 0.0001;
    public static double maxPower = 1;
    public static double edgePower = 0.35;
    public static int edgeDistance = 400;

    private int targetPosition;

    private int startPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        motor = ahwMap.get(DcMotor.class, "IntakeSlidesMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = motor.getCurrentPosition();
        targetPosition = 0;
        PIDF = new PIDController(kP, kI, kD, aTimer);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        motor = ahwMap.get(DcMotor.class, "IntakeSlidesMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (isTeleop) {
            Integer position = AutoToTeleopContainer.getInstance().getIntakeSlidesPosition();
            if (position == null) {
                startPosition = motor.getCurrentPosition();
            } else { startPosition = position;}
        } else {
            startPosition = motor.getCurrentPosition();
            AutoToTeleopContainer.getInstance().setIntakeSlidesPosition(startPosition);
        }
        targetPosition = 0;
        PIDF = new PIDController(kP, kI, kD, aTimer);
    }

    public double getPosition() {return ((double)(motor.getCurrentPosition()-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public double getTargetPosition() {return ((double)(targetPosition-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public void setTargetPosition(double target) {
        targetPosition = (int)(target * (maxPosition-minPosition) + minPosition);
    }

    public void incrementTargetPosition(double increment) {
        targetPosition += (int)(increment * (maxPosition-minPosition) + minPosition);
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
    }

    @Override
    public void periodic() {
        int currentPosition = motor.getCurrentPosition();
        double powerMultThrottle = edgePower + (maxPower - edgePower) * Math.min(((double)(Math.min(Math.abs(currentPosition-minPosition), Math.abs(currentPosition-maxPosition))))/edgeDistance, 1.0);
        motor.setPower(Math.min(powerMultThrottle, Math.max(PIDF.update(motor.getCurrentPosition()-startPosition, targetPosition) * powerMultThrottle, -powerMultThrottle)));
    }

}
