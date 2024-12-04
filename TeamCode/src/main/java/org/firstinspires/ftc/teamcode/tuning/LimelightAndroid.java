package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp
public class LimelightAndroid extends LinearOpMode {

    Limelight3A limelight;
    LLResultTypes.CalibrationResult calibration;
    LLStatus status;
    LLResult result;
//    private AprilTagProcessor myAprilTag;
//    private AprilTagDetection targetTag;
    boolean targetFound = false;
    double xCor;
    double yCor;
    int tagId = -1; //Tag ID that we looking for; Set to -1 for ANY tag
    @Override
    public void runOpMode() throws  InterruptedException {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
            calibration = limelight.getCalDefault();
            status = limelight.getStatus();
            limelight.pipelineSwitch(0);
        } catch(Exception e) {
            telemetry.addLine("Limelight: Null");
            telemetry.update();
        }
        telemetry.setMsTransmissionInterval(11);
        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {

            try {
                result = limelight.getLatestResult();
                telemetry.addLine("Result: Valid");
                telemetry.addLine();
                if(result.isValid()){
                    limelightStat();
                    telemetry.addData("Tx: ", "%.1f", result.getTx()); //Unit: Degree
                    telemetry.addData("Ty: ", "%.1f", result.getTy()); //Unit: Degree
                    telemetry.addData("Ta: ", "%.1f", result.getTa()); //Unit: Degree
                    telemetry.addLine();
                    Pose3D botpose = result.getBotpose();
                    double captureLatency = result.getCaptureLatency();
                    double targetingLatency = result.getTargetingLatency();
                    double parseLatency = result.getParseLatency();
                    telemetry.addData("LL Latency", captureLatency + targetingLatency);
                    telemetry.addData("Parse Latency", parseLatency);
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for(LLResultTypes.FiducialResult fr: fiducialResults){
                        telemetry.addData("Fiducial ID: ", fr.getFiducialId());
                        telemetry.addLine("Result Testing");
                        telemetry.update();
                    }
                }
            } catch (Exception e){
                telemetry.addLine("Result: Null");
            }
            telemetry.update();

            if(gamepad1.a){
                limelight.pipelineSwitch(1);
            }
            if (gamepad1.x){
                limelight.pipelineSwitch(0);
            }
            if(gamepad1.b){
                limelight.pipelineSwitch(2);
            }
        }
        sleep(5000);
        limelight.stop();
    }

    private void limelightStat(){
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline ", "Index: %d, Type: %s\n",
                status.getPipelineIndex(), status.getPipelineType());
        telemetry.addLine();
    }


}