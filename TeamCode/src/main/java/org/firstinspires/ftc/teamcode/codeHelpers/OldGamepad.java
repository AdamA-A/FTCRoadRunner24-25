package org.firstinspires.ftc.teamcode.codeHelpers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OldGamepad {
    // TODO implement a class for buttons, allowing for keypress, keydown, dbl-press, and other events, based off of web JavaScript events

    // A default interval threshold for updating gamepad
    static final double defaultMsThreshold = 500;

    Gamepad gamepad;
    ElapsedTime runtime;
    double lastUpdateMilliseconds;
    double millisecondThreshold;

    public OldGamepad(Gamepad initialGamepad) {
        this(initialGamepad, new ElapsedTime(), defaultMsThreshold);
    }

    public OldGamepad(Gamepad initialGamepad, double _millisecondThreshold) {
        this(initialGamepad, new ElapsedTime(), _millisecondThreshold);
    }

    public OldGamepad(Gamepad initialGamepad, ElapsedTime contextualRuntime) {
        this(initialGamepad, contextualRuntime, defaultMsThreshold);
    }

    public OldGamepad(Gamepad initialGamepad, ElapsedTime contextualRuntime, double _millisecondThreshold) {
        gamepad = initialGamepad;
        runtime = contextualRuntime;
        lastUpdateMilliseconds = runtime.milliseconds();
        millisecondThreshold = _millisecondThreshold;
    }

    public double millisecondsSinceLastUpdate() {
        return runtime.milliseconds() - lastUpdateMilliseconds;
    }

    public boolean update(Gamepad newGamepad) {
        if (millisecondsSinceLastUpdate() >= millisecondThreshold) {
            gamepad = newGamepad;
            return true;
        }
        return false;
    }
}
