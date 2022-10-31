package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Subsystem {
    protected ElapsedTime timer;
    protected double      startTime;
    protected HardwareMap hwMap;

    public Subsystem(ElapsedTime aTimer, HardwareMap ahwMap) {
        timer = aTimer;
        startTime = timer.milliseconds();
        hwMap = ahwMap;
    }

    public void update() {

    }

    public void shutDown() {

    }
}
