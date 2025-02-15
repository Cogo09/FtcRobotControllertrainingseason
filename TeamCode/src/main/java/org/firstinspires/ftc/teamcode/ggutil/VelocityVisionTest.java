package org.firstinspires.ftc.teamcode.ggutil;

import static org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.FRONT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.gentrifiedApps.velocityvision.classes.CameraParams;
import org.gentrifiedApps.velocityvision.classes.LensIntrinsics;
import org.gentrifiedApps.velocityvision.classes.RotationVector;
import org.gentrifiedApps.velocityvision.classes.TranslationalVector;
import org.gentrifiedApps.velocityvision.pipelines.homography.HomographicMatrix;
import org.gentrifiedApps.velocityvision.pipelines.homography.HomographicProjection;

@TeleOp
public class VelocityVisionTest extends LinearOpMode {
HomographicProjection homographicProjection = new HomographicProjection(new CameraParams(new LensIntrinsics(140.0,140.0,640.0,480.0),
        new TranslationalVector(0.0,0.0,0.0),new RotationVector(0.0,0.0,0.0)));

    private VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(homographicProjection)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal,60);
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
