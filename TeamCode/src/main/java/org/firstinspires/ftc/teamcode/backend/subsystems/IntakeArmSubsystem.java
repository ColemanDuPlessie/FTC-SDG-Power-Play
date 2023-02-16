package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.ArmPIDFController;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class IntakeArmSubsystem extends SubsystemBase implements PositionControlled {

    public ServoImpl servo;

    public static double minPosition = 0.4; // TODO this is way too conservative
    public static double maxPosition = 0.6; // TODO this is way too conservative
    public static double posIncrement = -0.02; // TODO this is way too conservative

    double targetPosition = 0.0;

    private int pingpongPosition = -1;
    private boolean pingpongReversed = false;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(ServoImpl.class, "IntakeArmServo");
        servo.setPosition(minPosition);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        init(aTimer, ahwMap);
    }

    public double getTargetPosition() {return targetPosition;}

    public double getPosition() {return (servo.getPosition()-minPosition)/(maxPosition-minPosition);}

    public void setTargetPosition(double target) {
        pingpongPosition = target == 1.0 ? 1 : -1;
        pingpongReversed = false;
        targetPosition = target;
        servo.setPosition(target * (maxPosition-minPosition) + minPosition);
    }

    public void extend() {setTargetPosition(1.0);}
    public void retract() {setTargetPosition(0.0);}
    public void hide() {setTargetPosition(0.5);} // TODO find good out-of-the-way setpoint

    public void incrementTargetPosition(double increment) {
        targetPosition += increment;
        targetPosition = Math.min(Math.max(targetPosition, 0.0), 1.0);
        servo.setPosition(targetPosition * (maxPosition-minPosition) + minPosition);
    }

    public void pingpong() {
        if (pingpongPosition == -1) {extend();} // not yet in a position to pingpong
        else if (pingpongReversed) {
            pingpongPosition--;
            incrementTargetPosition(-posIncrement);
            if (pingpongPosition == 1) {
                pingpongReversed = false;
            }
        } else {
            pingpongPosition++;
            incrementTargetPosition(posIncrement);
            if (pingpongPosition == 5) {
                pingpongReversed = true;
            }
        }
    }

}
