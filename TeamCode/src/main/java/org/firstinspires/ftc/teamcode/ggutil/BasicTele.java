package org.firstinspires.ftc.teamcode.ggutil;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.gentrifiedApps.gentrifiedAppsUtil.classExtenders.gamepad.FloatButton;
import org.gentrifiedApps.gentrifiedAppsUtil.classExtenders.gamepad.GamepadPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.DrivePowerCoefficients;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.MecanumDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.TankDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;

@TeleOp
public class BasicTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        LoopTimeController loopTimeController = new LoopTimeController();
        GamepadPlus gamepadPlus1 = new GamepadPlus(gamepad1, false);
        GamepadPlus gamepadPlus2 = new GamepadPlus(gamepad2, false);
        Driver driver = new Driver(this, "fl", "fr", "bl", "br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, null);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        driver.update();

        waitForStart();
        while (opModeIsActive()) {
//            DrivePowerCoefficients powerCoefficients = TankDriver.driveTankRobotCentric(gamepadPlus1.readFloat(FloatButton.LEFT_Y),gamepadPlus1.readFloat(FloatButton.RIGHT_X));
//            DrivePowerCoefficients powerCoefficients = TankDriver.driveTank(gamepadPlus1.readFloat(FloatButton.LEFT_Y),gamepadPlus1.readFloat(FloatButton.RIGHT_Y));
            DrivePowerCoefficients powerCoefficients = MecanumDriver.driveMecanum(gamepadPlus1.readFloat(FloatButton.LEFT_X), gamepadPlus1.readFloat(FloatButton.LEFT_Y), -gamepadPlus1.readFloat(FloatButton.RIGHT_X));

            driver.setWheelPower(powerCoefficients);

            loopTimeController.update();
            gamepadPlus1.sync();
            gamepadPlus2.sync();

            loopTimeController.telemetry(telemetry);

            telemetry.update();
        }
    }

}
