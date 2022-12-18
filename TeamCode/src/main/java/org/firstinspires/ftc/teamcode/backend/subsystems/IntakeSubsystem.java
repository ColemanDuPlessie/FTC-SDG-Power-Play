package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.ArmPIDFController;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class IntakeSubsystem extends SubsystemBase implements PositionControlled {

    public ServoImpl servo;

    public static double downPosition = 0.0;
    public static double upPosition = 0.3;


    private double targetPosition = downPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(ServoImpl.class, "IntakeServo");
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        servo = ahwMap.get(ServoImpl.class, "IntakeServo");
    }

    public double getTargetPosition() {return targetPosition;}

    public double getPosition() {return servo.getPosition();}

    public void setTargetPosition(double target) {
        targetPosition = target;
        servo.setPosition(targetPosition);
    }

    public void incrementTargetPosition(double increment) {
        targetPosition += targetPosition;
        targetPosition = Math.min(Math.max(targetPosition, 0.0), 1.0);
        servo.setPosition(targetPosition);
    }

    public void raise() {setTargetPosition(upPosition);}
    public void lower() {setTargetPosition(downPosition);}

}
