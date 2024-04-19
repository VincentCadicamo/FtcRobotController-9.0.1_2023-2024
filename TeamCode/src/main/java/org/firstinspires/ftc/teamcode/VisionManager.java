package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionManager {

    private static final String[] LABELS = {"RED_ELEMENT", "BLUE_ELEMENT", "BLUE_ELEMENT"};
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private static List<Recognition> currentRecognitions;
    private Telemetry telemetry;

    public VisionManager(HardwareMap hardwareMap, float minConfidence, Telemetry telemetry) {
        telemetry.addData("(VisionManager) Status: ", "Initializing...");
        telemetry.update();
        this.telemetry = telemetry;
        tfod = new TfodProcessor.Builder()
                .setModelFileName("/sdcard/FIRST/tflitemodels/NEW_TEAM_ELEMENT.tflite")
                .setModelAssetName("NEW_TEAM_ELEMENT.tflite")
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(tfod);
        builder.enableLiveView(true);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(minConfidence);
        tfod.setZoom(1.5);

        //visionPortal.setProcessorEnabled(tfod, true);
        currentRecognitions = tfod.getRecognitions();
        telemetry.addData("(VisionManager) Status: ", "<Ready>");
        telemetry.addData("(VisionManager) # of Recognitions: ", currentRecognitions.size());
        telemetry.update();
        }

        //Self Explanatory methods
        public void updateVision() {
            currentRecognitions = tfod.getRecognitions();
            telemetry.addData("(VisionManager) # of Recognitions: ", currentRecognitions.size());
            telemetry.update();
        }

        public int getNumberOfDetections() {
            updateVision();
            return currentRecognitions.size();
        }

        public List<Recognition> getRecognitions() {
            updateVision();
            return currentRecognitions;
        }

        public PixelCoordinate getPixelCoordinate(int index) {
            updateVision();
            if (index >= 0 && index < currentRecognitions.size()) {
                if (currentRecognitions.get(index).getLabel().equals("Pixel")) {
                    Recognition recognition = currentRecognitions.get(index);
                    return new PixelCoordinate(((recognition.getLeft() + recognition.getRight()) / 2), ((recognition.getTop() + recognition.getBottom()) / 2));
                }
            }

            return null;
        }

        public boolean determineTeamElement() {
            updateVision();
            List<Recognition> recognitions = getRecognitions();
            if (recognitions.size() > 1) {

            } else if (recognitions.size() > 0 && ( recognitions.get(0).getLabel().equals("RED_ELEMENT") || recognitions.get(0).getLabel().equals("RED_ELEMENT"))) return true;
            return false;
        }



    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = ((recognition.getTop()  + recognition.getBottom()) / 2) ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
    }   // end method telemetryTfod()
}