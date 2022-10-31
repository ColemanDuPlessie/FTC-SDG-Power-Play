package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.Subsystem;

@Config
public class CarouselSubsystem extends Subsystem implements SpeedControlled { // TODO

    protected DcMotor carouselMotor;

    private double speed = 0;

    public CarouselSubsystem(ElapsedTime aTimer, HardwareMap ahwMap) {
        super(aTimer, ahwMap);
        carouselMotor = hwMap.get(DcMotor.class, "DuckMotor");
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void setSpeed(double speed) {
        carouselMotor.setPower(speed);
        this.speed = speed;
    }

    @Override
    public double getSpeed() {
        return speed;
    }

}
