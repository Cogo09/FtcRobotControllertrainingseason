package org.firstinspires.ftc.teamcode.ggutil;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.drive.DrivePowerCoefficients;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.pointClasses.Point;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;

@TeleOp
public class AvoidanceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AvoidanceController avoidanceController = new AvoidanceController(new VectorField(0,0,10));
        Driver driver = new Driver(this, "fl", "fr", "bl", "br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Scribe.getInstance().startLogger(this);
        waitForStart();
        while (opModeIsActive()) {
            DrivePowerCoefficients coefs = avoidanceController.update(new Point(0,5));
            Scribe.getInstance().logDebug("Coefficients:"+ coefs.toString());
//            driver.setWheelPower(coefs);

            telemetry.update();
        }
    }

}
