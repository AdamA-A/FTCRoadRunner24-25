package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

import java.util.Arrays;

@TeleOp
public class Configurator extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Filler array for concatTwoStringArrays
        final String[] emptyStrArr = {};

        // Get motor names
        String[] driveMotors = {"rightFront", "rightBack", "leftFront", "leftBack"};
        final String[] motors = concatTwoStringArrays(Arm.configMotorNames, driveMotors);

        // Get servo names
        final String[] servos = {};

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Test motors
            telemetry.addLine("Motors");
            telemetry.addLine();
            for (String motor: motors) {
                telemetry.addData(motor, exists(motor, true));
                telemetry.addLine();
            }

            // Test servos
            telemetry.addLine();
            telemetry.addLine("Servos");
            for (String servo: servos) {
                telemetry.addLine();
                telemetry.addData(servo, exists(servo, false));
            }
            // TODO: use telemetry.addAction to add a runnable action of saving the current configuration to a file (so find if controlhub/expansionhub), and maybe use the logging object of hardwareMap

            telemetry.update();
        }
    }
    private String[] concatTwoStringArrays(String[] first, String[] second) {
        String[] both = Arrays.copyOf(first, first.length + second.length);
        System.arraycopy(second, 0, both, first.length, second.length);
        return both;
    }
    // Tests if motor (isMotor) or servo (!isMotor) exists
    private String exists(String name, boolean isMotor) {
        try {
            int port = -1;
            if (isMotor) {
                DcMotor testMotor = hardwareMap.dcMotor.get(name);
                port = testMotor.getPortNumber();
            } else {
                Servo testServo = hardwareMap.servo.get(name);
                port = testServo.getPortNumber();;
            }
            return "found at port " + port;
        } catch(Exception e) {
            return "not found";
        }
    }
}