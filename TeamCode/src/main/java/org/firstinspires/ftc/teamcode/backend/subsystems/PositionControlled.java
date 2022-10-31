package org.firstinspires.ftc.teamcode.backend.subsystems;

public interface PositionControlled extends HasPosition {

    double getPosition();

    void setTargetPosition(double target);

}
