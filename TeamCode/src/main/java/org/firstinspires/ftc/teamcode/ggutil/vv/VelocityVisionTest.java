package org.firstinspires.ftc.teamcode.ggutil.vv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.gentrifiedApps.gentrifiedAppsUtil.velocityVision.classes.CameraParams;
import org.gentrifiedApps.gentrifiedAppsUtil.velocityVision.classes.RotationVector;
import org.gentrifiedApps.gentrifiedAppsUtil.velocityVision.classes.TranslationalVector;

@TeleOp
public class VelocityVisionTest extends LinearOpMode {
    HomographicProjectionTester homographicProjection = new HomographicProjectionTester(new CameraParams(new LensIntrinsicsImpl(140.0, 140.0, 640.0, 480.0),
            new TranslationalVector(0.0, 0.0, 0.0), new RotationVector(45.0, 0.0, 0.0)));

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(homographicProjection)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 60);
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
