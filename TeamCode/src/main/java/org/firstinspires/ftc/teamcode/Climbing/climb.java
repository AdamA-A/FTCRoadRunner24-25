package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class MachsAwesomeCode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
    // Declare drivetrain motors
    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

    // Declare auxiliary motors
    DcMotor hexMotor = hardwareMap.dcMotor.get("hexMotor");
    DcMotor hexm = hardwareMap.dcMotor.get("hexm");

    // Declare the Continuous Rotation Servo
    CRServo smartServo = hardwareMap.get(CRServo.class, "smartServo");

    // Reverse right-side drivetrain motors for consistent movement direction
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

    if (isStopRequested()) return;

    while (opModeIsActive()) {
    // Drive control
    double y = -gamepad1.left_stick_y; // Forward is reversed on the Y stick
    double x = gamepad1.left_stick_x * 1.1; // Adjust for strafing imperfection
    double rx = gamepad1.right_stick_x;

    // Mecanum wheel calculations
    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double frontLeftPower = (y + x + rx) / denominator;
    double backLeftPower = (y - x + rx) / denominator;
    double frontRightPower = (y - x - rx) / denominator;
    double backRightPower = (y + x - rx) / denominator;

    // Set drivetrain motor powers
    frontLeftMotor.setPower(frontLeftPower);
    backLeftMotor.setPower(backLeftPower);
    frontRightMotor.setPower(frontRightPower);
    backRightMotor.setPower(backRightPower);

    // Hex motor control using B and Y buttons
    if (gamepad1.b) {
    hexMotor.setPower(1.0);
    hexm.setPower(-1.0);  // Opposite direction for synchronized action
} else if (gamepad1.y) {
    hexMotor.setPower(-1.0);
    hexm.setPower(1.0);   // Opposite direction for synchronized action
} else {
    hexMotor.setPower(0.0);
    hexm.setPower(0.0);   // Stop the motors when neither button is pressed
}

// Continuous Rotation Servo control using A and X buttons
if (gamepad1.a) {
    smartServo.setPower(1.0);  // Forward at full speed
} else if (gamepad1.x) {
    smartServo.setPower(-1.0); // Reverse at full speed
} else {
    smartServo.setPower(0.0);  // Stop the servo
}
}
}
}
