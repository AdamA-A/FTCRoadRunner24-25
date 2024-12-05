package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Combined Arm and Intake", group = "Test")
public class INTAKE_CLIMB extends LinearOpMode {

    private CRServo intakeLeft;
    private CRServo intakeRight;
    private Servo wrist;
    private Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize intake components
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        wrist = hardwareMap.get(Servo.class, "intakeWrist");

        // Initialize the Arm subsystem
        arm = Arm.getInstance(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // Intake Controls
            if (gamepad1.left_bumper) {
                intakeLeft.setPower(1.0);
                intakeRight.setPower(-1.0);
                wrist.setPosition(0.0); // Adjust wrist position as needed
            } else if (gamepad1.right_bumper) {
                intakeLeft.setPower(-1.0);
                intakeRight.setPower(1.0);
                wrist.setPosition(1.0); // Adjust wrist position as needed
            } else {
                intakeLeft.setPower(0.0);
                intakeRight.setPower(0.0);
            }

            // Arm Controls
            // Extend Arm to 90% when holding `Y`, retract to 0% when `X` is pressed
            if (gamepad1.y) {
                arm.extendToPercent(90, 1.0); // Extend to 90% at full power
            } else if (gamepad1.x) {
                arm.extendToPercent(0, -1.0); // Retract to 0% at full power
            }

            // Rotate Arm based on button input
            if (gamepad1.a) {
                arm.rotateToPosition(0, 1.); // Rotate to rest
            } else if (gamepad1.b) {
                arm.rotateToPosition(-50, -1.); // Rotate bucket scoring position
            } else {
                arm.holdPosition(); // Hold rotational position when no input
            }

            // Telemetry for debugging
            telemetry.addData("Intake Left Power", intakeLeft.getPower());
            telemetry.addData("Intake Right Power", intakeRight.getPower());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("EXTEND Position", arm.getExtendPosition());
            telemetry.addData("PIVOT Position", arm.getPivotPosition());
            telemetry.update();
        }
    }

    // Define the Arm class
    public static class Arm {
        private DcMotor rotationalMotor;
        private DcMotor extendMotor;
        private static Arm instance;

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

        public void extendToPercent(double percent, double power) {
            int targetPosition = (int) (percent / 100 * 3300); // Assuming 3300 ticks is 100%
            extendMotor.setTargetPosition(targetPosition);
            extendMotor.setPower(power);
        }

        public void rotateToPosition(int position, double power) {
            rotationalMotor.setTargetPosition(position);
            rotationalMotor.setPower(power);
        }

        public void holdPosition() {
            rotationalMotor.setTargetPosition(rotationalMotor.getCurrentPosition());
            rotationalMotor.setPower(0.1); // Low power to hold position
        }

        public int getExtendPosition() {
            return extendMotor.getCurrentPosition();
        }

        public int getPivotPosition() {
            return rotationalMotor.getCurrentPosition();
        }
    }
}
