package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class Climb extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare auxiliary motors
        DcMotor pivot = hardwareMap.dcMotor.get("climbRight");
        DcMotor pivot2 = hardwareMap.dcMotor.get("climbLeft");
        // Set both pivot motors to brake mode for holding position when power is zero
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set motors to use encoders
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Define the target position
        final int CLIMB_TARGET_POSITION = 1000; // Adjust this value based on your robot setup
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            // Button A: Set the climb motors to a specific target position
            if (gamepad1.a) {
                pivot.setTargetPosition(CLIMB_TARGET_POSITION);
                pivot2.setTargetPosition(-CLIMB_TARGET_POSITION); // Opposite direction if needed
                // Switch to RUN_TO_POSITION mode
                pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivot2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // Set power to start moving to the target position
                pivot.setPower(0.7);
                pivot2.setPower(0.7);
            }
            // Button B: Move the motors in opposite directions at full speed
            else if (gamepad1.b) {
                pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pivot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pivot.setPower(1.0);
                pivot2.setPower(-1.0);
            }
            // Stop the motors when no button is pressed
            else {
                pivot.setPower(0.0);
                pivot2.setPower(0.0);
            }
            // Telemetry for debugging
            telemetry.addData("Climb Right Position", pivot.getCurrentPosition());
            telemetry.addData("Climb Left Position", pivot2.getCurrentPosition());
            telemetry.update();
        }
    }
}