package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.ArmPIDFController;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class DepositSubsystem extends SubsystemBase {

    public CRServo left;
    public CRServo right;

    private double currentSpeed = 0.0;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        left = ahwMap.get(CRServo.class, "LeftDeposit");
        right = ahwMap.get(CRServo.class, "RightDeposit");
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setPower(currentSpeed);
        right.setPower(currentSpeed);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        this.init(aTimer, ahwMap);
    }

    public double getCurrentSpeed() {return currentSpeed;}

    public void setSpeed(double speed) {
        currentSpeed = Math.min(Math.max(speed, -1.0), 1.0);
        left.setPower(currentSpeed);
        right.setPower(currentSpeed);
    }

    public void intake() {setSpeed(1.0);}
    public void hold() {setSpeed(0.0);}
    public void deposit() {setSpeed(-1.0);}

    public void toggleIntake() {
        if (getCurrentSpeed() == 1.0) {setSpeed(0.0);
        } else {setSpeed(1.0);}
    }
    public void toggleDeposit() {
        if (getCurrentSpeed() == -1.0) {setSpeed(0.0);
        } else {setSpeed(-1.0);}
    }
}
