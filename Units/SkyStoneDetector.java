package org.firstinspires.ftc.teamcode.Units;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class SkyStoneDetector {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "ARvqM1n/////AAAAGU135bpUmE4nhYqzaGDNYANxXsz/hxo6ZkWyy88ce4IlnLxC7t3fUdComptOI2CFcV2f+NzESuJgyDVHYaRDI/KCl0MY/HkEyuQlxpY1EZ1XaOG162ZLxcqohgh96C8jiMJH6DDhR90xhOD1zps/shW5iNolUfzTv3FxlHclhcY/64KrUP9oT5zlHLRR90LJ2uo9IesckSfwcHF04GAheSqbF84PZFkYgIgtlIbjZx5nZS3Qrpw2ggmcZe1qemzit88XIBNBlHwIzwC+26TjkxIyNFAIyDSCK9fNDlTpBO3PlWgZt5btmnY+sz62F7WqgwQPwhHl61FlKuz0ss7bZLFh/iQZMEWiMCKa5aWGjgKb";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private OpMode opMode;

    public enum Position {
        LEFT,
        CENTER,
        RIGHT
    }

    public void init(OpMode opMode) {
        this.opMode = opMode;

        initVuforia();
        initTfod();

        //CameraDevice.getInstance().setFlashTorchMode(true);
        CameraDevice.getInstance().setField("opti-zoom", "opti_zoom_on");
        CameraDevice.getInstance().setField("zoom", "15");

        if (tfod != null)
            tfod.activate();
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public Position getSkystonePosition() {
        LinearOpMode linearOpMode = (LinearOpMode)opMode;

        if (linearOpMode.opModeIsActive() && (tfod != null)) {
            Double position = null;
            List<Recognition> updatedRecognitions;

            while (linearOpMode.opModeIsActive() && position == null) {
                updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null)
                    for (Recognition recognition : updatedRecognitions)
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT))
                            if (position == null)
                                position = recognition.getLeft() + recognition.getWidth() / 2.0;
                            else if (position < (recognition.getLeft() + recognition.getWidth() / 2.0))
                                position = recognition.getLeft() + recognition.getWidth() / 2.0;
            }

            if (position == null)
                return Position.CENTER;

            if (position < 505)
                return Position.LEFT;
            else if (position < 725)
                return Position.CENTER;
            else
                return Position.RIGHT;
        }

        return Position.CENTER;
    }

    public void shutdown() {
        tfod.shutdown();
    }
}
