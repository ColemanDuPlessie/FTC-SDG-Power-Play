package org.firstinspires.ftc.teamcode.backend.utilities;

import java.util.function.Supplier;

public class PressableButton implements Runnable {

    protected final Supplier<Boolean> getter;
    protected final Runnable effect;
    private boolean isAlreadyDown;

    public PressableButton(Supplier<Boolean> buttonGetter, Runnable buttonEffect) {
        getter = buttonGetter;
        effect = buttonEffect;
        isAlreadyDown = false;
    }

    public void update() {
        if (getter.get() != isAlreadyDown) {
            isAlreadyDown = !isAlreadyDown;
            if (isAlreadyDown) {effect.run();}
        }
    }

    @Override
    public void run() {
        this.update();
    }
}
