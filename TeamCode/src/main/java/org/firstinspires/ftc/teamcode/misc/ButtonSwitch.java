package org.firstinspires.ftc.teamcode.misc;

public class ButtonSwitch {
    private boolean lastButtonstate = false;
    private boolean trigger = false;

    public boolean isTriggered(boolean buttonState) {
        trigger = ((buttonState != lastButtonstate) && buttonState) != trigger;
        lastButtonstate = buttonState;
        return trigger;
    }
}

