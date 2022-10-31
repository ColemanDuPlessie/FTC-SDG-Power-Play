package org.firstinspires.ftc.teamcode.backend.utilities;

import com.qualcomm.robotcore.hardware.Servo;

public class DoubledServo {

    private Servo[] servos = new Servo[2];
    private boolean areReversed = false;

    public DoubledServo(Servo servo1, Servo servo2) {
        servos[0] = servo1;
        servos[1] = servo2;
    }

    public DoubledServo(Servo servo1, Servo servo2, boolean areReversed) {
        this(servo1, servo2);
        this.areReversed = areReversed;
        servo2.setDirection(areReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        for (Servo servo : servos) {
            servo.setPosition(position);
        }
    }

}
