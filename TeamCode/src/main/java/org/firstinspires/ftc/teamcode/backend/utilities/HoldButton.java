package org.firstinspires.ftc.teamcode.backend.utilities;

import java.util.function.Supplier;

public class HoldButton implements ButtonWrapper, Supplier<Boolean> {

    private final Supplier<Boolean> getter;

    public HoldButton(Supplier<Boolean> buttonGetter) {
        getter = buttonGetter;
    }

    public boolean update() {
        return getter.get();
    }

    public Boolean get() {return update();}

}
