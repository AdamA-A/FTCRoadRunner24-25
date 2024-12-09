package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DRIVE extends LinearOpMode {
    private CRServo intakeLeft, intakeRight;
    private Servo wrist;
    private Arm arm;

    // States for the control logic
    private enum ArmState {
        IDLE,
        PICKING,
        OUTTAKING
    }

    private ArmState armState = ArmState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors and servos
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        wrist = hardwareMap.servo.get("Wrist");
        arm = Arm.getInstance(hardwareMap);

        // Reverse right side motors for proper forward movement
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x;  // Strafing
            double rx = gamepad1.right_stick_x; // Rotation

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Adjust for robot rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Motor power calculations
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Handle arm state machine
            switch (armState) {
                case IDLE:
                    // Intake controls
                    if (gamepad1.left_bumper) {
                        intakeLeft.setPower(1.0);
                        intakeRight.setPower(-1.0);
                        wrist.setPosition(0.0);
                    } else if (gamepad1.right_bumper) {
                        intakeLeft.setPower(-1.0);
                        intakeRight.setPower(1.0);
                        wrist.setPosition(1.0);
                    } else {
                        intakeLeft.setPower(0.0);
                        intakeRight.setPower(0.0);
                    }

                    // Transition to PICKING state
                    if (gamepad1.x) {
                        armState = ArmState.PICKING;
                        arm.extendToPercentPID(20, 0.5);
                    }

                    // Transition to OUTTAKING state
                    if (gamepad1.y) {
                        armState = ArmState.OUTTAKING;
                        arm.rotateToPositionPID(-100, -0.7);
                        arm.extendToPercentPID(40, 0.5);
                    }
                    break;

                case PICKING:
                    wrist.setPosition(0.5);
                    intakeLeft.setPower(-1.0);
                    intakeRight.setPower(1.0);

                    if (gamepad2.b) {
                        intakeLeft.setPower(0.0);
                        intakeRight.setPower(0.0);
                        arm.extendToPercentPID(1, -0.5);
                        armState = ArmState.IDLE;
                    }
                    break;

                case OUTTAKING:
                    wrist.setPosition(0.0);
                    intakeLeft.setPower(-0.25);
                    intakeRight.setPower(0.25);

                    if (gamepad2.b) {
                        intakeLeft.setPower(0.0);
                        intakeRight.setPower(0.0);
                        wrist.setPosition(1.0);
                        arm.rotateToPositionPID(, 0.7);
                        arm.extendToPercentPID(1, 0.5);
                        armState = ArmState.IDLE;
                    }
                    break;
            }

            // Debugging telemetry
            telemetry.addData("State", armState);
            telemetry.addData("Left Power", frontLeftPower);
            telemetry.addData("Right Power", frontRightPower);
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Arm Extend Position", arm.getExtendPosition());
            telemetry.addData("Arm Pivot Position", arm.getPivotPosition());
            telemetry.update();
        }
    }

    public static class Arm {
        private DcMotor rotationalMotor, extendMotor;
        private static Arm instance;

        private Arm(HardwareMap hardwareMap) {
            rotationalMotor = hardwareMap.dcMotor.get("rotationalMotor");
            extendMotor = hardwareMap.dcMotor.get("extendMotor");

            rotationalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rotationalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public static Arm getInstance(HardwareMap hardwareMap) {
            if (instance == null) {
                instance = new Arm(hardwareMap);
            }
            return instance;
        }

        public void extendToPercentPID(double percent, double power) {
            int targetPosition = (int) (percent / 100 * 3300); // Assuming 3300 ticks is 100%
            extendMotor.setTargetPosition(targetPosition);
            extendMotor.setPower(power);
        }

        public void rotateToPositionPID(int position, double power) {
            rotationalMotor.setTargetPosition(position);
            rotationalMotor.setPower(power);
        }

        public int getExtendPosition() {
            return extendMotor.getCurrentPosition();
        }

        public int getPivotPosition() {
            return rotationalMotor.getCurrentPosition();
        }
    }
}
