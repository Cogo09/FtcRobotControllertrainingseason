package org.firstinspires.ftc.teamcode.cgutil.CAMSUBS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.teamcode.cgutil.CAMUTIL.CAMDUALENCODER;
import org.firstinspires.ftc.teamcode.cgutil.CAMUTIL.CAMPIDVALS;

import java.util.List;

public class CAMARMSUB {
    DcMotor hangmotor = null;
    DcMotor hangmotor1 = null;
    public static CAMDUALENCODER dualEncoder = null;
    double power = 0;
    double power1 = 0;
    double uptarget = 0;
    PIDFController uppidf = new PIDFController(0, 0, 0, 0);
    PIDFController uppidf1 = new PIDFController(0, 0, 0, 0);

    boolean auto;

    //this is where you put all enums and variables
    public CAMARMSUB(HardwareMap hwMap, Boolean auto) {
        this.auto = auto;
        hangmotor = hwMap.dcMotor.get("hangmotor");
        hangmotor1 = hwMap.dcMotor.get("hangmotor1");
//        hangmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        hangmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dualEncoder = new CAMDUALENCODER(hangmotor, hangmotor1);

//        hangmotor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        hangmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
        updatePID();
        setPowerup(uptarget);
        power();
    }

    // this is where you put your update functions to switch between states
    public void telemetry(Telemetry telemetry) {
        dualEncoder.telemetry(telemetry);
    }

    private void updatePID() {
        uppidf.setPIDF(
                CAMPIDVALS.upco.p,
                CAMPIDVALS.upco.i,
                CAMPIDVALS.upco.d,
                CAMPIDVALS.upco.f
        );
        uppidf1.setPIDF(
                CAMPIDVALS.upco1.p,
                CAMPIDVALS.upco1.i,
                CAMPIDVALS.upco1.d,
                CAMPIDVALS.upco1.f
        );
    }

    private double calculatePID(PIDFController controller, double currentVal, double target) {
        return Range.clip(
                controller.calculate(
                        currentVal,
                        target
                ), -1, 1);

    }

    void power() {
        hangmotor.setPower(power);
        hangmotor1.setPower(power1);
    }

    public void setUptarget(double target) {
        uptarget = target;
    }

    private void setPowerup(double target) {
        power = calculatePID(uppidf, hangmotor.getCurrentPosition(), target);
        power1 = calculatePID(uppidf1, -hangmotor1.getCurrentPosition(), target);//getmost?
    }

    public boolean isUpAtTarget(double tolerance) {
        double most = dualEncoder.getmost();
        return (uptarget - tolerance < most) && (most < uptarget + tolerance);
    }


    public Action armAction(List<Runnable> funcs) {
        return new CAMARMSUB.ArmAction(funcs);
    }

    class ArmAction implements Action {
        List<Runnable> funcs;

        public ArmAction(List<Runnable> funcs) {
            this.funcs = funcs;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (Runnable func : funcs) {
                func.run();
            }
            return false;
        }
    }
}