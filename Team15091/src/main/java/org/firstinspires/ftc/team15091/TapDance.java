package org.firstinspires.ftc.team15091;

public class TapDance {
    public int tapMs = 500;
    private long timer = System.currentTimeMillis();
    private boolean button_pressed = false;
    private int numTaps = 0;
    public int lastTapResult = 0;
    public int getTapCount (boolean button) {
        if (!button_pressed && button) {
            button_pressed = true;
            if (System.currentTimeMillis() - timer < tapMs) {
                numTaps++;
                lastTapResult = numTaps;
                return numTaps;
            }
            else {
                numTaps = lastTapResult = 1;
                return 1;
            }
        }
        else if (button_pressed && !button) {
            button_pressed = false;
            timer = System.currentTimeMillis();
        }
        lastTapResult = 0;
        return 0;
    }

    public void resetTapCount () {
        numTaps = 0;
        lastTapResult = 0;
        timer = 0;
    }
}
