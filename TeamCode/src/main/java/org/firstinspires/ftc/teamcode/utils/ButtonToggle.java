package org.firstinspires.ftc.teamcode.utils;

public class ButtonToggle {
    private boolean lastButtonPosition = false;
    public boolean isOn = false;
    public boolean update(boolean buttonPressed) {
        if (lastButtonPosition == false && buttonPressed == true) {
            return true;
        }
        return false;
    }
}
