package org.firstinspires.ftc.teamcode.ggutil;

import static org.firstinspires.ftc.teamcode.ggutil.pidTuners.PIDTuning.armMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.dacodingbeast.pidtuners.Simulators.AngleRange;
import com.dacodingbeast.pidtuners.utilities.DataLogger;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.motor.PIDMotor;

import CommonUtilities.PIDParams;

@TeleOp
@Config
public class PIDManualTuning extends LinearOpMode {
    public static double kP =0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    PIDMotor motor = null;
    public static double target = 200;
    public static double target2 = 45;
    DcMotor motor2;
//   PIDFcontroller controller;

    @Override
    public void runOpMode() throws InterruptedException {
//        motor2 = hardwareMap.get(DcMotor.class, "pivot");
//        controller = new PIDFcontroller(new PIDParams(kP, kI, kD, kF), false);
        motor = new PIDMotor(hardwareMap,"pivot", DcMotorSimple.Direction.REVERSE).currentReversed();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor.setPIDF(kP, kI, kD, kF);
        motor.setTarget(target);
        waitForStart();
        while (opModeIsActive()) {
//            controller
//                    .setParams(new PIDParams(kP, kI, kD, kF));
//            motor2.setPower(controller.calculate(AngleRange.fromDegrees(findPose(motor2.getCurrentPosition()), target2), null).getMotorPower());
            motor.setPIDF(kP, kI, kD, kF);
            double current = motor.getCurrentPosition();
            telemetry.addData("current",current);
            telemetry.addData("target",motor.getTarget());
            Scribe.getInstance().logData("current"+current);
            Scribe.getInstance().logData("target"+motor.getTarget());
            telemetry.update();
            if (gamepad1.dpad_up){
                target = 200;
                //pidController.calculate(current, end); = power
            }else if (gamepad1.dpad_down){
                target = 0;
            }
            motor.setTarget(target);
            motor.setPIDPower();
        }
    }
    double findPose(double current){
        double angle = AngleRange.wrap((current * (2 * Math.PI / armMotor.getMotorSpecs().getEncoderTicksPerRotation())));
        DataLogger.getInstance().logData(angle);
        return angle;
    }
}
