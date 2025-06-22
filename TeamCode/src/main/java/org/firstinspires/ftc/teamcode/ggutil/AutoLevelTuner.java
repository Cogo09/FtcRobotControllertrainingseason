package org.firstinspires.ftc.teamcode.ggutil;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.gentrifiedApps.gentrifiedAppsUtil.classes.equations.SlopeIntercept;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.kinematics.AutoLevel1DF;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.GamepadPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.ServoPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;

@TeleOp
public class AutoLevelTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ServoPlus servo = new ServoPlus(this.hardwareMap, "servo");
        AutoLevel1DF autoLevel = new AutoLevel1DF(0.5, -30.0, new SlopeIntercept(0.5, 20));

        LoopTimeController loopTimeController = new LoopTimeController();
        GamepadPlus gamepadPlus1 = new GamepadPlus(gamepad1);
        GamepadPlus gamepadPlus2 = new GamepadPlus(gamepad2);
        Driver driver = new Driver(this, "fl", "fr", "bl", "br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            autoLevel.setServoPosition(60,servo);

//            DrivePowerCoefficients powerCoefficients = TankDriver.driveTankRobotCentric(gamepadPlus1.readFloat(FloatButton.LEFT_Y),gamepadPlus1.readFloat(FloatButton.RIGHT_X));
//            DrivePowerCoefficients powerCoefficients = TankDriver.driveTank(gamepadPlus1.readFloat(FloatButton.LEFT_Y),gamepadPlus1.readFloat(FloatButton.RIGHT_Y));
//            DrivePowerCoefficients powerCoefficients = MecanumDriver.driveMecanum(gamepadPlus1.readFloat(FloatButton.LEFT_X), gamepadPlus1.readFloat(FloatButton.LEFT_Y), -gamepadPlus1.readFloat(FloatButton.RIGHT_X));
//
//            driver.setWheelPower(powerCoefficients);
//
//            loopTimeController.update();
//            gamepadPlus1.sync();
//            gamepadPlus2.sync();
//
//            loopTimeController.telemetry(telemetry);

            telemetry.update();
        }
    }

}
