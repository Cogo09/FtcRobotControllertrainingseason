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
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.Button;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.FloatButton;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.GamepadPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;

import java.util.HashMap;

@TeleOp
public class ChildTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LoopTimeController loopTimeController = new LoopTimeController();
        GamepadPlus gamepadPlus1 = new GamepadPlus(gamepad1, false);
        GamepadPlus gamepadPlus2 = new GamepadPlus(gamepad2, false);

        Driver driver = new Driver(this, "fl", "fr", "bl", "br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        HashMap<Enum<?>, SlowModeMulti> slowModeMap = new HashMap<>();
        slowModeMap.put(SMD.SLOW_MODE, new SlowModeMulti(SlowMode.of(2.0),Button.B));
        SlowModeManager slowModeManager = new SlowModeManager(slowModeMap,gamepadPlus1);

        telemetry.update();
        driver.update();

        Scribe.getInstance().startLogger("ChildTele");
        waitForStart();

        loopTimeController.setLoopSavingCache(hardwareMap);
        while (opModeIsActive()) {
            slowModeManager.setCurrentlyActive(SMD.SLOW_MODE);
            DrivePowerCoefficients powerCoefficients = MecanumDriver.driveMecanum(gamepadPlus1.readFloat(FloatButton.LEFT_X),gamepadPlus1.readFloat(FloatButton.LEFT_Y),gamepadPlus1.readFloat(FloatButton.RIGHT_X));

            driver.setWheelPower(powerCoefficients.applySlowMode(slowModeManager));

            loopTimeController.update();

            loopTimeController.telemetry(telemetry);
            driver.update();
            gamepadPlus1.sync();
            gamepadPlus2.sync();
        }
    }
}
