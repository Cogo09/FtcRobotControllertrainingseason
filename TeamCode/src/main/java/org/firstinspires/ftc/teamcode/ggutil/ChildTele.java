package org.firstinspires.ftc.teamcode.ggutil;

import android.util.Pair;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.SlowMode;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.SlowModeDefaults;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.SlowModeManager;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.SlowModeMulti;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.Timeout;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.AnalogEncoder;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.Operand;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.Operation;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.DrivePowerCoefficients;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.driverAid.DriverAid;
import org.gentrifiedApps.gentrifiedAppsUtil.dataStorage.DataStorage;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.FieldCentricDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.MecanumDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.Button;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.FloatButton;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.GamepadPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.ServoPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.Angle;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.AngleUnit;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.Target2D;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.localizers.tracking.MecanumLocalizer;
import org.gentrifiedApps.gentrifiedAppsUtil.idler.Idler;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;
import org.gentrifiedApps.gentrifiedAppsUtil.motion.controllers.SquIDController;
import org.gentrifiedApps.gentrifiedAppsUtil.motion.profiles.SlewRateLimiter;
import org.gentrifiedApps.gentrifiedAppsUtil.motion.profiles.TrapezoidalMotionProfile;
import org.gentrifiedApps.gentrifiedAppsUtil.sensorArray.Sensor;
import org.gentrifiedApps.gentrifiedAppsUtil.sensorArray.SensorArray;

import java.lang.annotation.Target;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

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
