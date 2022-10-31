package org.firstinspires.ftc.teamcode.backend.utilities;

import java.util.function.Supplier;

public class PressableHoldButton extends PressableButton implements Runnable {

    public PressableHoldButton(Supplier<Boolean> buttonGetter, Runnable buttonEffect) {
        super(buttonGetter, buttonEffect);
    }

    public void update() {
        if (getter.get()) {effect.run();}
    }

    public void run() {
        this.update();
    }
}
