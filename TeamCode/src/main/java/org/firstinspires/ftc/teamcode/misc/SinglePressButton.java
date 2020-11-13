package org.firstinspires.ftc.teamcode.misc;

public class SinglePressButton {
    private boolean lastButtonstate = false;
    public boolean isTriggered(boolean buttonState) {
        boolean trigger = ((buttonState != lastButtonstate) && buttonState);
        lastButtonstate = buttonState;
        return trigger;
    }
}
