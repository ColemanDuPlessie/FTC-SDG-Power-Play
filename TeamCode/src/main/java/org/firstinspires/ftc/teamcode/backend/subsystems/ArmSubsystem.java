package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.ArmPIDFController;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class ArmSubsystem extends SubsystemBase implements PositionControlled {

    public DcMotor motor;

    private PIDController PIDF;

    public static int minPosition = 0;
    public static int maxPosition = 1600;
    public static int horizPos = 100;
    public static int vertPos = 750;

    public static double kP = 0.015;
    public static double kI = 0.0001;
    public static double kD = 0.0002;
    public static double kG = 0.65;
    public static double powerMultThrottle = 0.5;

    private int targetPosition;

    private int startPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        motor = ahwMap.get(DcMotor.class, "ArmMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = motor.getCurrentPosition()-maxPosition/2;
        targetPosition = startPosition-maxPosition/10;
        PIDF = new ArmPIDFController(kP, kI, kD, aTimer, kG, horizPos, vertPos);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        motor = ahwMap.get(DcMotor.class, "ArmMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (isTeleop) {
            Integer position = AutoToTeleopContainer.getInstance().getArmPosition();
            if (position == null) {
                startPosition = motor.getCurrentPosition()-maxPosition/2;
            } else { startPosition = position;}
        } else {
            startPosition = motor.getCurrentPosition()-maxPosition/2;
            AutoToTeleopContainer.getInstance().setArmPosition(startPosition);
        }
        targetPosition = startPosition-maxPosition/10;
        PIDF = new ArmPIDFController(kP, kI, kD, aTimer, kG, horizPos, vertPos);
    }

    public double getTargetPosition() {return ((double)(targetPosition-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public double getPosition() {return ((double)(motor.getCurrentPosition()-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public void setTargetPosition(double target) {
        targetPosition = (int)(target * (maxPosition-minPosition) + minPosition);
    }

    public void incrementTargetPosition(double increment) {
        targetPosition += (int)(increment * (maxPosition-minPosition) + minPosition);
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
    }

    @Override
    public void periodic() {
        motor.setPower(Math.min(powerMultThrottle, Math.max(PIDF.update(motor.getCurrentPosition()-startPosition, targetPosition) * powerMultThrottle, -powerMultThrottle)));
    }

}
