package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClimbPOOOO extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors and servos
        DcMotor climbRight = hardwareMap.dcMotor.get("climbRight");
        DcMotor climbLeft = hardwareMap.dcMotor.get("climbLeft");
        Servo Servo_CLimb_Left = hardwareMap.servo.get("hookLeft");
        Servo Servo_CLimb_Right = hardwareMap.servo.get("hookRight");

        // Set motors to brake when power is zero
        climbRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configurable gear ratio and TPR calculation
        final double GEAR_RATIO = 125.0; // Current gear ratio
        final double TICKS_PER_REVOLUTION = 28 * GEAR_RATIO;
        final double CLIMB_REVOLUTIONS = 15;
        double CLIMBTicks = CLIMB_REVOLUTIONS * TICKS_PER_REVOLUTION;

        // Servo and motor configurations
        final double SERVO_RIGHT_START_POSITION = .75; // Fully retracted
        final double SERVO_RIGHT_PREP_POSITION = (SERVO_RIGHT_START_POSITION + .5);// Fully extended
        final double SERVO_LEFT_START_POSITION = -0.7175;// Fully retracted
        final double SERVO_LEFT_PREP_POSITION = (SERVO_LEFT_START_POSITION + 1.);// Fully extended

        // Reset motor encoders
        climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        climbRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                // Button A: Climb Prep
                Servo_CLimb_Left.setPosition(SERVO_LEFT_PREP_POSITION);
                Servo_CLimb_Right.setPosition(SERVO_RIGHT_PREP_POSITION );

                // Convert revolutions to ticks and move motors
                climbRight.setTargetPosition((int) CLIMBTicks);
                climbLeft.setTargetPosition((int) CLIMBTicks);

                climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                climbRight.setPower(-1.0);
                climbLeft.setPower(-1.0);

            } else if (gamepad1.b) {
                // Button B: Climbing Action
                Servo_CLimb_Left.setPosition(SERVO_LEFT_START_POSITION);
                Servo_CLimb_Right.setPosition(SERVO_RIGHT_START_POSITION);

                // Convert revolutions to ticks and move motors
                climbRight.setTargetPosition((int) CLIMBTicks);
                climbLeft.setTargetPosition((int) CLIMBTicks);

                climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                climbRight.setPower(1.0);
                climbLeft.setPower(1.0);

            } else {
                // Stop motors if no buttons are pressed
                climbRight.setPower(0);
                climbLeft.setPower(0);
            }

            // Telemetry for debugging
            double climbRightRevolutions = climbRight.getCurrentPosition() / TICKS_PER_REVOLUTION;
            double climbLeftRevolutions = climbLeft.getCurrentPosition() / TICKS_PER_REVOLUTION;

            telemetry.addData("Climb Right Revolutions", climbRightRevolutions);
            telemetry.addData("Climb Left Revolutions", climbLeftRevolutions);
            telemetry.addData("servo Left Position", Servo_CLimb_Left.getPosition());
            telemetry.addData("Servo Right Position", Servo_CLimb_Right.getPosition());
            telemetry.update();
        }
    }
}
