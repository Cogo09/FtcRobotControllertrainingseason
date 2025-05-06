package org.firstinspires.ftc.teamcode.cgutil.CAMHARDWARE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cgutil.CAMSUBS.CAMCLAWSUB;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.drive.DrivePowerCoefficients;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.MecanumDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.firstinspires.ftc.teamcode.cgutil.CAMSUBS.CAMARMSUB;
//import org.gentrifiedApps.gentrifiedAppsUtil.LoopTimeController;

public class HARDConfig {
    boolean slowmode = false;
    Telemetry telemetry = null;
    LinearOpMode opMode = null;
    public CAMCLAWSUB clawsub = null;
    public CAMARMSUB armSub = null;
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    Driver driver = null;

    //    Limelight3A limelight = null;
//    DcMotor armMotor1 = null;
    ElapsedTime elapsedTime = null;
    //LoopTimeController loopTimeController = null;

    public HARDConfig(LinearOpMode om, HardwareMap hwmap, Boolean auto) {
        initrobot(hwmap, om, auto);
    }

    void initrobot(HardwareMap hwmap, LinearOpMode om, Boolean auto) {
        opMode = om;//
        telemetry = om.telemetry;
        clawsub = new CAMCLAWSUB(hwmap);
        armSub = new CAMARMSUB(hwmap, auto);
        driver = new Driver(opMode,"frontLeftMotor", "frontRightMotor", "backLeftMotor", "backRightMotor", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);


        frontLeftMotor = hwmap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hwmap.dcMotor.get("backLeftMotor");
        frontRightMotor = hwmap.dcMotor.get("frontRightMotor");
        backRightMotor = hwmap.dcMotor.get("backRightMotor");
        //armMotor1 = hwmap.dcMotor.get("armMotor1");
        // limelight = hwmap.get(Limelight3A.class, "limelight");
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
        // armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        elapsedTime = new ElapsedTime();
        //loopTimeController = new LoopTimeController(elapsedTime, null);
    }

    public void buildtelemetry() {
        telemetry.addData("slowmode", slowmode);
        telemetry.addData("claw", opMode.gamepad1.right_bumper);
//        telemetry.addData("b", opMode.gamepad1.b);
//        telemetry.addData("up", opMode.gamepad1.dpad_up);
        //telemetry.addData("armout", armMotor1.getCurrentPosition());
        telemetry.addData("rightstick", opMode.gamepad2.right_stick_y);
        //armSub.telemetry(telemetry);
        //loopTimeController.telemetry(telemetry);
        telemetry.update();
    }

    boolean touchpadwpressed = false;

    public void dobulk() {


        DrivePowerCoefficients powerCoefficients = MecanumDriver.driveMecanum(opMode.gamepad1.left_stick_x, -opMode.gamepad1.right_stick_x,opMode.gamepad1.left_stick_y);
        boolean touchpadpressed = opMode.gamepad1.touchpad;
        if (touchpadpressed && !touchpadwpressed) {
            slowmode = !slowmode;
        }
        touchpadwpressed = touchpadpressed;
        double slowmodemultiplier = 2;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double multiplier = 1;
        if (slowmode) {
            multiplier = slowmodemultiplier;
        }


//        armMotor1.setPower(armpower);
        clawsub.update();
        //loopTimeController.update();
        armSub.update();
        driver.setWheelPower(powerCoefficients.applySlowMode(multiplier));

        buildtelemetry();

    }
}
