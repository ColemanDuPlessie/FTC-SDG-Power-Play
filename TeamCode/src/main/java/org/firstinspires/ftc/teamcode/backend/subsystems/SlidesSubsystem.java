package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.GravityPIDFController;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class SlidesSubsystem extends SubsystemBase implements PositionControlled {

    public DcMotor motor;
    private DcMotor followerMotor;

    private PIDController PIDF;

    public static int minPosition = 0; // We don't actually want to go all the way down.
    public static int maxPosition = 2850;

    public static double kP = 0.007;
    public static double kI = 0.0000;
    public static double kD = 0.000065;
    public static double kG = 0.25;
    public static double maxPower = 0.75;
    public static double edgePower = 0.25;
    public static int edgeDistance = 400;
    public static double overallMultiplier = 0.8;

    private int targetPosition;

    private int startPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        motor = ahwMap.get(DcMotor.class, "SlidesMotor");
        followerMotor = ahwMap.get(DcMotor.class, "SlidesFollowerMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        followerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = motor.getCurrentPosition();
        targetPosition = 0;
        PIDF = new PIDController(kP, kI, kD, aTimer);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        motor = ahwMap.get(DcMotor.class, "SlidesMotor");
        followerMotor = ahwMap.get(DcMotor.class, "SlidesFollowerMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        followerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        PIDF = new PIDController(kP, kI, kD, aTimer);
    }

    public double getPosition() {return ((double)(motor.getCurrentPosition()-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public double getTargetPosition() {return ((double)(targetPosition-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public void setTargetPosition(double target) {
        targetPosition = (int)(target * (maxPosition-minPosition) + minPosition);
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
    }

    public void incrementTargetPosition(double increment) {
        targetPosition += (int)(increment * (maxPosition-minPosition) + minPosition);
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
    }

    @Override
    public void periodic() {
        int currentPosition = motor.getCurrentPosition();
        double powerMultThrottle = edgePower + (maxPower - edgePower) * Math.min(((double)(Math.abs(currentPosition-targetPosition)))/edgeDistance, 1.0);
        double actualPower = Math.min(powerMultThrottle, Math.max(PIDF.update(motor.getCurrentPosition()-startPosition, targetPosition) * powerMultThrottle, -powerMultThrottle)) + kG;
        motor.setPower(actualPower*overallMultiplier);
        followerMotor.setPower(actualPower*overallMultiplier);
    }

}
