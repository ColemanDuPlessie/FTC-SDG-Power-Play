package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.CommandScheduler;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;

import java.util.ArrayList;
import java.util.HashMap;

public class AutoToTeleopContainer {
    private static final AutoToTeleopContainer INSTANCE = new AutoToTeleopContainer();
    public static AutoToTeleopContainer getInstance() {return INSTANCE;}

    public Double forwardsAngleDelta;
    private Integer slidesPosition;
    private Integer armPosition;

    private AutoToTeleopContainer() {
    }

    public void setAngleDelta(double toSet) {forwardsAngleDelta = toSet;}

    public Double getAngleDelta() {
        if (forwardsAngleDelta == null) {return null;}
        double ans = forwardsAngleDelta;
        forwardsAngleDelta = null;
        return ans;
    }

    public void setSlidesPosition(int toSet) {slidesPosition = toSet;}

    public Integer getSlidesPosition() {
        if (slidesPosition == null) {return null;}
        int ans = slidesPosition;
        slidesPosition = null;
        return ans;
    }

    public void setArmPosition(int toSet) {armPosition = toSet;}

    public Integer getArmPosition() {
        if (armPosition == null) {return null;}
        int ans = armPosition;
        armPosition = null;
        return ans;
    }

}
