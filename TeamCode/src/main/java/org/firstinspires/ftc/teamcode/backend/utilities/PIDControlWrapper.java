package org.firstinspires.ftc.teamcode.backend.utilities;

import org.firstinspires.ftc.teamcode.backend.subsystems.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

public class PIDControlWrapper implements PositionControlled {

    private final AcceptsPIDController plant;
    private final PIDController controller;
    private double targetPosition = 0;

    public PIDControlWrapper(AcceptsPIDController plant, PIDController controller) {
        this.plant = plant;
        this.controller = controller;
    }

    @Override
    public double getPosition() {
        return plant.getPosition();
    }

    public void update() {
        double currPos = plant.getPosition();
        plant.setSpeed(controller.update(plant.getPosition(), targetPosition));
    }

    @Override
    public void setTargetPosition(double target) {
        targetPosition = target;
    }
}
