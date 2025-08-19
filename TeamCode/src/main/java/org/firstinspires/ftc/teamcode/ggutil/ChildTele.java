package org.firstinspires.ftc.teamcode.ggutil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.drive.DrivePowerCoefficients;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.SlowMode;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.SlowModeManager;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.SlowModeMulti;

import org.gentrifiedApps.gentrifiedAppsUtil.drive.MecanumDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;

import java.util.HashMap;

@TeleOp
public class ChildTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LoopTimeController loopTimeController = new LoopTimeController();

        Driver driver = new Driver(this, "fl", "fr", "bl", "br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        SlowMode slowMode = new SlowMode(2.0);
        telemetry.update();

        Scribe.getInstance().startLogger("ChildTele");
        //

        loopTimeController.setLoopSavingCache(hardwareMap);
        while (opModeIsActive()) {
            DrivePowerCoefficients powerCoefficients = MecanumDriver.driveMecanum(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

            driver.setWheelPower(slowMode.apply(powerCoefficients));

            loopTimeController.update();

            loopTimeController.telemetry(telemetry);
        }
    }
}
