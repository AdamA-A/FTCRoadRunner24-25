package org.firstinspires.ftc.teamcode.tuning;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class Adam extends LinearOpMode {
    /* UPDATED ON 10/24/2024 */

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        // Declare the REV Hex motor
        //DcMotor hexMotor = hardwareMap.dcMotor.get("hexMotor");
        // Declare the REV Smart Robot Servo
        //Servo smartServo = hardwareMap.get(Servo.class, "smartServo");
        // Reverse the right side motors if needed
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            // Drive control
            double y = -gamepad1.left_stick_y; // Y stick is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // Calculate motor powers for mecanum drive
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            // Control the REV Hex motor with the B button
        }
    }
}