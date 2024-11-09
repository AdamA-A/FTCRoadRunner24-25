package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    // Counts per revolution
    final double CPR = 28;
    final double MAX_EXTENSION_CM = 96.69 - 41.15;
    final double MAX_EXTENSION_TICKS = 3300;
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
        rotationalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setTargetPosition(0);
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extendBeforeMaxBy(double cm, double extendPower) {
        extendTo(MAX_EXTENSION_CM - cm);
        extendMotor.setPower(extendPower);
    }

    public void extendToPercent(double percent, double extendPower) {
        double cm = MAX_EXTENSION_CM * percent / 100;
        extendTo(cm);
        extendMotor.setPower(extendPower);
    }

    private void extendTo(double cm) {

        // Convert angle to position
        int position = cmToTicks(cm);

        // Set Position
        extendMotor.setTargetPosition(position);
    }

    public void rotateTo(double angle, boolean toNormalize) {

        // Convert angle to position
        int position = angleToTicks((toNormalize ? angle % 360 : angle));

        // Set Position
        rotationalMotor.setTargetPosition(position);
    }


    // Starting length = 41.15cm at 0 ticks
    // Ending length = 96.69cm at 3300 ticks
    private int cmToTicks(double cm) {
        double exactTicks = cm * MAX_EXTENSION_TICKS/MAX_EXTENSION_CM;
        return (int) exactTicks;
    }
    private int angleToTicks(double angle) {
        double revolutions = angle / 360;
        double exactPosition = revolutions * CPR;
        return (int) exactPosition;
    }
}
