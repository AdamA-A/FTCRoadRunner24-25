package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class DRIVE extends LinearOpMode {
    private CRServo intakeLeft;
    private Servo wrist;
    private Arm arm;
    private IMU imu;
    private CRServo intakeRight;
    private CRServo climbTurnerLeft;
    private CRServo climbTurnerRight;
    private Climb climb;

    // States for the control logic
    private enum ArmState {
        IDLE,
        INTAKING,
        OUTTAKING
    }

    private ArmState armState = ArmState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors and servos

        // DRIVE
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        // INTAKE
        intakeLeft = hardwareMap.crservo.get("intakeLeft");
        intakeRight = hardwareMap.crservo.get("intakeRight");
        wrist = hardwareMap.servo.get("Wrist");

        // CLIMB
        climbTurnerLeft = hardwareMap.crservo.get("LeftClimbServo");
        climbTurnerRight = hardwareMap.crservo.get("RightClimbServo");
        climb = new Climb(hardwareMap); // Initialize Climb object

        imu = hardwareMap.get(IMU.class, "imu");
        arm = Arm.getInstance(hardwareMap);

        // Reverse the right-side motors for proper driving
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Driving logic
            double y = -gamepad1.left_stick_y; // Inverted Y axis
            double x = gamepad1.left_stick_x * 1.1; // Adjusted strafing
            double rx = gamepad1.right_stick_x; // Rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Arm and intake state machine
            switch (armState) {
                case IDLE:
                    if (gamepad1.left_bumper) {
                        intakeLeft.setPower(-1.0);
                        intakeRight.setPower(1.0);
                        wrist.setPosition(1.0);
                    } else if (gamepad1.right_bumper) {
                        intakeLeft.setPower(1.0);
                        intakeRight.setPower(-1.0);
                        wrist.setPosition(0.650);
                    }

                    if (gamepad1.x) {
                        armState = ArmState.INTAKING;
                    }

                    if (gamepad1.y) {
                        arm.rotateToPositionPID(-600, -0.25);
                        armState = ArmState.OUTTAKING;
                    }

                    // Climbing logic
                    if (gamepad2.y) {
                        climb.moveToPosition(1000, -1.0);
                        climbTurnerRight.setPower(-1.0);
                        climbTurnerLeft.setPower(-1.0);
                    }

                    if (gamepad2.b) {
                        climb.moveToPosition(0, 1.0);
                        climbTurnerRight.setPower(0.0);
                        climbTurnerLeft.setPower(0.0);
                    }
                    break;

                case INTAKING:
                    wrist.setPosition(1);
                    intakeLeft.setPower(-0.5);
                    intakeRight.setPower(0.5);
                    arm.extendToPercentPID(0.4, 0.5);

                    if (gamepad1.left_bumper) {
                        wrist.setPosition(0);
                    }

                    if (gamepad2.b) {
                        intakeLeft.setPower(0.0);
                        intakeRight.setPower(0.0);
                        arm.extendToPercentPID(0.01, -0.5);
                        armState = ArmState.IDLE;
                    }
                    break;

                case OUTTAKING:
                    wrist.setPosition(0.650);
                    arm.extendToPercentPID(0.85, 0.25);

                    if (gamepad1.a) {
                        intakeLeft.setPower(0.25);
                        intakeRight.setPower(-0.25);
                    }

                    if (gamepad2.b) {
                        intakeLeft.setPower(0.0);
                        intakeRight.setPower(0.0);
                        wrist.setPosition(1.0);

                        while (!gamepad2.x) {
                            arm.extendToPercentPID(0.01, -0.5);
                        }

                        arm.rotateToPositionPID(1.0, 0.1);
                        armState = ArmState.IDLE;
                    }
                    break;
            }

            // Telemetry updates
            telemetry.addData("Heading (Degrees)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("State", armState);
            telemetry.addData("Left Power", frontLeftPower);
            telemetry.addData("Right Power", frontRightPower);
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Arm Extend Position", arm.getExtendPosition());
            telemetry.addData("Arm Pivot Position", arm.getPivotPosition());
            telemetry.addData("Left Intake Power", intakeLeft.getPower());
            telemetry.addData("Right Intake Power", intakeRight.getPower());
            climb.displayTelemetry(telemetry);
            telemetry.update();
        }
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

        public void extendToPercentPID(double percent, double maxPower) {
            if (percent < 0) percent = 0;
            if (percent > 1) percent = 1;

            int targetPosition = (int) (3000 * percent);
            extendMotor.setTargetPosition(targetPosition);
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double currentPosition = extendMotor.getCurrentPosition();
            double error = targetPosition - currentPosition;
            double power = Math.min(Math.abs(error) / 1000, maxPower);
            extendMotor.setPower(power);
        }

        public void rotateToPositionPID(double targetPosition, double maxPower) {
            rotationalMotor.setTargetPosition((int)targetPosition);
            rotationalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double currentPosition = rotationalMotor.getCurrentPosition();
            double error = targetPosition - currentPosition;
            double power = Math.min(Math.abs(error) / 1000, maxPower);
            rotationalMotor.setPower(power);
        }

        public int getExtendPosition() {
            return extendMotor.getCurrentPosition();
        }

        public int getPivotPosition() {
            return rotationalMotor.getCurrentPosition();
        }
    }

    public class Climb {
        private DcMotor climbRight, climbLeft;

        public Climb(HardwareMap hardwareMap) {
            // Initialize the motors
            climbRight = hardwareMap.dcMotor.get("climbRight");
            climbLeft = hardwareMap.dcMotor.get("climbLeft");

            // Reset encoders and set motor modes
            climbRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            climbLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            climbRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            climbLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Set zero power behavior
            climbRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            climbLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Method to move the motors to a specific position
        public void moveToPosition(int ticks, double power) {
            // Set target positions
            climbRight.setTargetPosition(ticks);
            climbLeft.setTargetPosition(ticks);

            // Set to RUN_TO_POSITION mode
            climbRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set motor power
            climbRight.setPower(power);
            climbLeft.setPower(power);
        }

        // Method to stop the motors
        public void stop() {
            climbRight.setPower(0);
            climbLeft.setPower(0);
        }

        // Method to reverse the motors
        public void reverseToPosition(int ticks, double power) {
            moveToPosition(-ticks, power);
        }

        // Telemetry for debugging
        public void displayTelemetry(Telemetry telemetry) {
            telemetry.addData("ClimbRight Position", climbRight.getCurrentPosition());
            telemetry.addData("ClimbLeft Position", climbLeft.getCurrentPosition());
            telemetry.addData("ClimbRight Power", climbRight.getPower());
            telemetry.addData("ClimbLeft Power", climbLeft.getPower());
        }
    }

}
