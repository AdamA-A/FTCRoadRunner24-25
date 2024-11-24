package org.firstinspires.ftc.teamcode.codeHelpers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Incrementor {
    // A default interval threshold for updating gamepad
    private static final double defaultMsThreshold = 500;

    private double value;
    private ElapsedTime runtime;
    private double lastUpdateMilliseconds;
    private double millisecondThreshold;

    public Incrementor(double _value) {
        this(_value, new ElapsedTime(), defaultMsThreshold);
    }

    public Incrementor(double _value, double _millisecondThreshold) {
        this(_value, new ElapsedTime(), _millisecondThreshold);
    }

    public Incrementor(double _value, ElapsedTime contextualRuntime) {
        this(_value, contextualRuntime, defaultMsThreshold);
    }

    public Incrementor(double _value, ElapsedTime contextualRuntime, double _millisecondThreshold) {
        value = _value;
        runtime = contextualRuntime;
        lastUpdateMilliseconds = runtime.milliseconds();
        millisecondThreshold = _millisecondThreshold;
    }

    private double millisecondsSinceLastUpdate() {
        return runtime.milliseconds() - lastUpdateMilliseconds;
    }

    private boolean canUpdate(double threshold) {
        return millisecondsSinceLastUpdate() >= threshold;
    }

    public double get() {
        return value;
    }

    public void set(double newValue) {
        value = newValue;
    }

    public void changeBy(double amount) {
        changeBy(amount, millisecondThreshold);
    }

    public boolean changeBy(double amount, double msThreshold) {
        if (canUpdate(msThreshold)) {
            value += amount;
            return true;
        }
        return false;
    }
}
