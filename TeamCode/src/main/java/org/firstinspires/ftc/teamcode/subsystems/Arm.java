package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    // Counts per revolution
    final double CPR = 28;
    final public static String[] configMotorNames = {"rotationalMotor", "extendMotor"};
    public static DcMotor rotationalMotor = null;
    public static DcMotor extendMotor = null;
    private static Arm instance = null;

    public static Arm getInstance(HardwareMap hMap) {
        return instance = instance == null ? new Arm(hMap) : instance;
    }

    private Arm(HardwareMap hMap) {
        hardwareMap = hMap;
        rotationalMotor = hardwareMap.dcMotor.get(Arm.configMotorNames[0]);
        extendMotor = hardwareMap.dcMotor.get(Arm.configMotorNames[1]);

        rotationalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationalMotor.setTargetPosition(0);
        rotationalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setTargetPosition(0);
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extendToMax() {
        extendTo(96.6);
        extendMotor.setPower(1.);
    }

    public void rotateTo(double angle, boolean toNormalize) {

        // Convert angle to position
        int position = angleToTicks((toNormalize ? angle % 360 : angle));

        // Set Position
        rotationalMotor.setTargetPosition(position);
    }

    private void extendTo(double cm) {

        // Convert angle to position
        int position = cmToTicks(cm);

        // Set Position
        extendMotor.setTargetPosition(position);
    }
    // Starting length = 41.15cm at 0 ticks
    // Ending length = 96.69cm at 3300 ticks (3374 ticks is the max, but we only should extend to 96.6cm)
    // ticks = cm * n
    // 3300 ticks = (96.69cm - 41.15cm) * n
    // solved for n, n = 3300/55.54

    // RE-EVALUATED AT 3306 ticks = (97cm - 41.15cm) * n
    // Corrected equation to: 3306 ticks = (97cm) * n
    // n = 3306/97
    private int cmToTicks(double cm) {
        double exactTicks = cm * (3306.0 / 97);
        return (int) exactTicks;
    }
    private int angleToTicks(double angle) {
        double revolutions = angle / 360;
        double exactPosition = revolutions * CPR;
        return (int) exactPosition;
    }
}
