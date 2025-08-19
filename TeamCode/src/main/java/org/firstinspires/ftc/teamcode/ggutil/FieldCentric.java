package org.firstinspires.ftc.teamcode.ggutil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.drive.DrivePowerCoefficients;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.pointClasses.Angle;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.SlowMode;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.FieldCentricDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.MecanumDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;

@TeleOp
public class FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LoopTimeController loopTimeController = new LoopTimeController();

        Driver driver = new Driver(this, "fl", "fr", "bl", "br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        telemetry.update();
        SlowMode slowMode = new SlowMode(2.0);

        Scribe.getInstance().startLogger("FieldCentric");
        waitForStart();

        loopTimeController.setLoopSavingCache(hardwareMap);
        while (opModeIsActive()) {
            DrivePowerCoefficients powerCoefficients = FieldCentricDriver.driveFieldCentric(-gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x, Angle.ofDegrees(imu.getRobotYawPitchRollAngles().getYaw()),Angle.ofDegrees(-90.0));

            driver.setWheelPower(powerCoefficients);

            loopTimeController.update();

            loopTimeController.telemetry(telemetry);
        }
    }
}
