package org.firstinspires.ftc.teamcode.ggutil;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Heatseeker;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generators.EncoderSpecsBuilder;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.Encoder;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.EncoderStorage;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.PIDController;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.PathBuilder;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.Angle;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.AngleUnit;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.Target2D;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.Waypoint;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.localizers.tracking.IMUParams;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.localizers.tracking.TwoWheelLocalizer;

import java.util.List;

@Autonomous
public class AutoTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        TwoWheelLocalizer localizer = new TwoWheelLocalizer(
                hardwareMap,
                new Target2D(0, 0, new Angle(0, AngleUnit.RADIANS)),
                new EncoderStorage(new Encoder(EncoderSpecsBuilder.INSTANCE.goBildaSwingArm(), "fl", DcMotorSimple.Direction.FORWARD, 0.0, hardwareMap), null
                        , new Encoder(EncoderSpecsBuilder.INSTANCE.goBildaSwingArm(), "fr", DcMotorSimple.Direction.FORWARD, 0.0, hardwareMap)),
                new IMUParams("imu", new IMU.Parameters(orientationOnRobot))
        );
        Driver driver = new Driver(this, "fl", "fr", "bl", "br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, localizer);
        Heatseeker heatseeker = new Heatseeker(driver,new PIDController(1.0,0.0,0.0),new PIDController(1.0,0.0,0.0),new PIDController(1.0,0.0,0.0));
        List<Waypoint> waypoints = new PathBuilder()
                .addWaypoint(new Waypoint(new Target2D(0, 10.0, new Angle(0, AngleUnit.RADIANS)), 0.5))
                .build();

        waitForStart();
        if (opModeIsActive()) {
            heatseeker.followPath(waypoints, 1);
        }
    }
}
