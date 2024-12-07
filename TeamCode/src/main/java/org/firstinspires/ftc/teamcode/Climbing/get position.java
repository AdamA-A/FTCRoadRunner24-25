package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Integrated TeleOp with Field-Centric Drive", group = "TeleOp")
public class IntegratedTeleOp extends LinearOpMode {

    private CRServo intakeLeft;
    private CRServo intakeRight;
    private Servo wrist;
    private Arm arm;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize intake components
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        wrist = hardwareMap.get(Servo.class, "intakeWrist");

        // Initialize the Arm subsystem
        arm = Arm.getInstance(hardwareMap);

        // Initialize the IMU for field-centric driving
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Initialize drive motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse right-side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Field-centric mecanum drive
            double y = -gamepad1.left_stick_y; // Y is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate joystick inputs by the robot's heading
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // A + B Button: Extend to 40%, Intake, Retract
            if (gamepad1.a && gamepad1.b) {
                arm.extendToPercentPID(40); // Extend arm to 40%

                // Start intake
                intakeLeft.setPower(-1.0);
                intakeRight.setPower(1.0);

                // Wait for Gamepad 2's B button to stop intake and retract
                while (!gamepad2.b && opModeIsActive()) {
                    telemetry.addData("Waiting for Gamepad 2 B", true);
                    telemetry.update();
                    idle();
                }

                intakeLeft.setPower(0.0);
                intakeRight.setPower(0.0);

                // Retract arm
                arm.extendToPercentPID(0);
            }

            // Y Button: Rotate, Extend, Outtake, Retract
            if (gamepad1.y) {
                arm.rotateToPositionPID(-700); // Rotate arm to position
                arm.extendToPercentPID(90);   // Extend arm to 90%

                // Outtake
                wrist.setPosition(0.0);
                intakeLeft.setPower(-0.25);
                intakeRight.setPower(0.25);

                // Wait for Gamepad 2's B button to stop outtake
                while (!gamepad2.b && opModeIsActive()) {
                    telemetry.addData("Outtaking...", true);
                    telemetry.update();
                    idle();
                }

                intakeLeft.setPower(0.0);
                intakeRight.setPower(0.0);
                wrist.setPosition(1.0);

                // Retract sequence
                arm.rotateToPositionPID(0);
                arm.extendToPercentPID(0);
            }

            // Telemetry
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Intake Left Power", intakeLeft.getPower());
            telemetry.addData("Intake Right Power", intakeRight.getPower());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Arm Extend Pos", arm.getExtendPosition());
            telemetry.addData("Arm Pivot Pos", arm.getPivotPosition());
            telemetry.update();
        }
    }

    public static class Arm {
        private DcMotor rotationalMotor;
        private DcMotor extendMotor;
        private static Arm instance;

        private static final double KP = 0.01, KI = 0.0001, KD = 0.005;

        private Arm(HardwareMap hardwareMap) {
            rotationalMotor = hardwareMap.dcMotor.get("rotationalMotor");
            extendMotor = hardwareMap.dcMotor.get("extendMotor");

            rotationalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotationalMotor.setTargetPosition(0);
            rotationalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendMotor.setTargetPosition(0);
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public static Arm getInstance(HardwareMap hardwareMap) {
            if (instance == null) {
                instance = new Arm(hardwareMap);
            }
            return instance;
        }

        public void extendToPercentPID(double percent) {
            int targetPosition = (int) (percent / 100 * 3300); // Assuming 3300 ticks is max
            pidControl(extendMotor, targetPosition);
        }

        public void rotateToPositionPID(int targetPosition) {
            pidControl(rotationalMotor, targetPosition);
        }

        private void pidControl(DcMotor motor, int targetPosition) {
            double integral = 0, previousError = 0;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive()) {
                int currentPosition = motor.getCurrentPosition();
                double error = targetPosition - currentPosition;
                integral += error;
                double derivative = error - previousError;

                double power = KP * error + KI * integral + KD * derivative;
                motor.setPower(Math.max(-1.0, Math.min(1.0, power))); // Clamp power
                previousError = error;

                if (Math.abs(error) < 10) { // Close enough
                    motor.setPower(0);
                    break;
                }

                telemetry.addData("Motor Target", targetPosition);
                telemetry.addData("Motor Current", currentPosition);
                telemetry.addData("Motor Error", error);
                telemetry.addData("Motor Power", power);
                telemetry.update();
            }

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public int getExtendPosition() {
            return extendMotor.getCurrentPosition();
        }

        public int getPivotPosition() {
            return rotationalMotor.getCurrentPosition();
        }
    }
}
