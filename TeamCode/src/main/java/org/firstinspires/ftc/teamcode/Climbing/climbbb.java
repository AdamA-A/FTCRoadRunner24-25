package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class CLIMBMEOOOOW extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare auxiliary motors
        DcMotor climbRight = hardwareMap.dcMotor.get("climbRight");
        DcMotor climbLeft = hardwareMap.dcMotor.get("climbLeft");
        DcMotor winch = hardwareMap.dcMotor.get("winchMotor");

        // Declare servos for hooks
        Servo hookLeft = hardwareMap.servo.get("hookLeft");
        Servo hookRight = hardwareMap.servo.get("hookRight");

        // Set both pivot motors to brake mode for holding position when power is zero
        climbRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Variables for climb positions and servo settings
        final int CLIMB_POSITION = 1000; // Example position; adjust as needed
        final double HOOK_DEPLOYED_POSITION = 1.0; // Fully deployed
        final double HOOK_RETRACTED_POSITION = 0.0; // Fully retracted

        // Reset encoders for climb motors
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run to position mode
        climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // Button A: Set climb to position
            if (gamepad1.a) {
                climbRight.setTargetPosition(CLIMB_POSITION);
                climbLeft.setTargetPosition(-CLIMB_POSITION);
                climbRight.setPower(0.7);
                climbLeft.setPower(0.7);
            } else if (gamepad1.b) {
                // Button B: Winch and deploy hooks
                winch.setPower(1.0);
                hookLeft.setPosition(HOOK_DEPLOYED_POSITION);
                hookRight.setPosition(HOOK_DEPLOYED_POSITION);
            } else {
                // Stop all motors if no button is pressed
                climbRight.setPower(0);
                climbLeft.setPower(0);
                winch.setPower(0);

                // Retract hooks
                hookLeft.setPosition(HOOK_RETRACTED_POSITION);
                hookRight.setPosition(HOOK_RETRACTED_POSITION);
            }

            // Telemetry for debugging
            telemetry.addData("Climb Right Position", climbRight.getCurrentPosition());
            telemetry.addData("Climb Left Position", climbLeft.getCurrentPosition());
            telemetry.addData("Hook Left Position", hookLeft.getPosition());
            telemetry.addData("Hook Right Position", hookRight.getPosition());
            telemetry.update();
        }
    }
}
