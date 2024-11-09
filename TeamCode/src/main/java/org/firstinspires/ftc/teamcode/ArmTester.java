package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp
public class ArmTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = Arm.getInstance(hardwareMap);
        boolean lastY = false;
        boolean driveMode = true;

        waitForStart();

        if (isStopRequested()) return; // 3 expansion hub extend, 1 control hub pivet

        double testExtendPercent = 0;

        while (opModeIsActive()) {
            // Add current runtime
            double now = getRuntime();
            telemetry.addData("Runtime", now);

            // Gamepad1's y is the switch for runtime
            if (gamepad1.y && lastY == false) {
                lastY = true;
                driveMode = !driveMode;
            } else if (gamepad1.y == false && lastY) {
                lastY = false;
            }

            // Behavior based on driveMode
            telemetry.addData("driveMode", driveMode);
            if (driveMode) {
                if (gamepad1.y) {
                    arm.extendToPercent(50, 1);
                }
            } else {
                if (gamepad1.a) {
                    arm.extendMotor.setPower(1.);
                    arm.extendMotor.setTargetPosition(-250);
                } else {
                    double y = -gamepad1.left_stick_y;
                    arm.extendToPercent(50 + (50 * y), 1);
                    telemetry.addData("Extending to percent", 50 + (50 * y));
                    telemetry.addData("y", y);
                }
                if (gamepad1.b) {
                    arm.rotationalMotor.setPower(0.2);
                    arm.rotationalMotor.setTargetPosition(1000);
                } else if (gamepad1.x) {
                    arm.rotationalMotor.setPower(-0.2);
                    arm.rotationalMotor.setTargetPosition(-1000);
                } else {
                    // TODO change power of 0.4 to 0, and fix brake mode; 0.4 causes motor to go in wrong direction at startup after falsifying driveMode
                    arm.rotationalMotor.setPower(0.4);
                    arm.rotationalMotor.setTargetPosition(arm.rotationalMotor.getCurrentPosition());
                }
            }
            // Show the position of the motor on telemetry
            telemetry.addData("EXTEND Position", arm.extendMotor.getCurrentPosition());
            telemetry.addData("EXTEND Target", arm.extendMotor.getTargetPosition());
            telemetry.addData("PIVOT Position", arm.rotationalMotor.getCurrentPosition());
            telemetry.addData("PIVOT Target", arm.rotationalMotor.getTargetPosition());
            telemetry.update();
        }
    }
}