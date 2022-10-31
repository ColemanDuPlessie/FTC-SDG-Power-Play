package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.utilities.AcceptsPIDController;

public class IntakeSubsystem extends Subsystem implements SpeedControlled, HasPosition, AcceptsPIDController {

    protected DcMotor intakeMotor;

    private final int startPosition;
    private double currSpeed = 0;

    public IntakeSubsystem(ElapsedTime aTimer, HardwareMap ahwMap) {
        super(aTimer, ahwMap);
        intakeMotor = hwMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = intakeMotor.getCurrentPosition();
    }

    public void setSpeed(double speed) {intakeMotor.setPower(speed); currSpeed = speed;}

    public double getSpeed() {return currSpeed;}

    public double getPosition() {return intakeMotor.getCurrentPosition() - startPosition;}
}
