package org.firstinspires.ftc.teamcode.util;

public class Toggle {

    private boolean lastInput = false;
    private boolean pressed = false;
    private boolean changed = false;
    private boolean state = false;

    public Toggle( ) { }
    public Toggle( boolean initialState ) { this.state = initialState; }

    public void update(boolean input) {
        if (lastInput != input) {
            changed = true;
            pressed = input;
            if (pressed) state = !state;
            lastInput = input;
        } else {
            changed = false;
        }
    }

    public void set(boolean input) {
        if (lastInput != input) {
            changed = true;
        } else {
            changed = false;
        }
        state = input;
        lastInput = input;
    }

    public boolean toggle() {
        state = !state;
        changed = true;
        return state;
    }

    public boolean isPressed() {
        return pressed;
    }

    public boolean isChanged() {
        return changed;
    }

    public boolean getState() { return state; }

    public boolean isClicked() {
        return changed && pressed;
    }

    public boolean isReleased() { return changed && !pressed; }
}
