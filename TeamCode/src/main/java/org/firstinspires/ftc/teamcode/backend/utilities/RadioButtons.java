package org.firstinspires.ftc.teamcode.backend.utilities;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

public class RadioButtons<T> implements Supplier<T>{

    private final Set<PressableButton> buttons = new HashSet<PressableButton>();
    private T currentState = null;
    private final boolean defaultToNull;

    public RadioButtons(HashMap<Supplier<Boolean>, T> buttonGetters, boolean defaultToNull) {
        for (Supplier<Boolean> button : buttonGetters.keySet()) {
            T buttonValue = buttonGetters.get(button);
            buttons.add(defaultToNull ? new PressableHoldButton(button, () -> {pressed(buttonValue);}) : new PressableButton(button, () -> {pressed(buttonValue);}));
        }
        this.defaultToNull = defaultToNull;
    }

    public T update() {
        /**
         * Returns the object associated with a button if that button was the one most recently
         * pressed (released is irrelevant). Defaults to the null value. If defaultToNull is true,
         * it will default to null if no button is actively being held. If multiple buttons are
         * being held simultaneously, behavior is undefined.
         */
        if (defaultToNull) {currentState = null;}
        for (PressableButton button : buttons) {button.update();}
        return currentState;
    }

    void pressed(T value) {currentState = value;}

    public T get() {return update();}

}
