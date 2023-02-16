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

    public static double closedPosition = 0.4; // TODO find setpoint
    public static double openPosition = 0.3; // TODO find setpoint
    public static double fullyOpenPosition = 0.2; // TODO find setpoint


    private double targetPosition = closedPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(ServoImpl.class, "IntakeServo");
        close();
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        servo = ahwMap.get(ServoImpl.class, "IntakeServo");
        close();
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

    public void close() {setTargetPosition(closedPosition);}
    public void open() {setTargetPosition(openPosition);}
    public void fullyOpen() {setTargetPosition(fullyOpenPosition);}

    public void toggle() {
        if (getTargetPosition() == openPosition) {
            close();
        } else {
            open();
        }
    }

}
