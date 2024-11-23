import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.List;
@TeleOp
public class LimelightConfig extends LinearOpMode {
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");

        } catch (Exception e) {
            telemetry.addLine("limelight is null");
        }
        waitForStart();

        if(isStopRequested()) return;
        while(opModeIsActive()){

        }
    }
}