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
        waitForStart();

        if (isStopRequested()) return; // 3 expansion hub extend, 1 control hub pivet

        while (opModeIsActive()) {
            if (gamepad1.y) {
                arm.extendToMax();
            } else if (gamepad1.a) {
                arm.extendMotor.setPower(1.);
                arm.extendMotor.setTargetPosition(-250);
            }
            if (gamepad1.b) {
                arm.rotationalMotor.setPower(0.25);
                arm.rotationalMotor.setTargetPosition(1000);
            } else if (gamepad1.x) {
                arm.rotationalMotor.setPower(-0.25);
                arm.rotationalMotor.setTargetPosition(-1000);
            } else {
                arm.rotationalMotor.setPower(0);
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