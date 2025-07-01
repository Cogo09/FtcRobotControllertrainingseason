package org.firstinspires.ftc.teamcode.cgutil.CAMAUTO;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cgutil.CAMHARDWARE.AUTOHARD;



@Autonomous
public class CAMAUTOTESTING extends LinearOpMode {
    AUTOHARD robot = null;
    ///
    ///
    ///
    ///
    ///
    ///
    ///THIS FILE IS ONLY FOR TESTING AUTONOMOUS OUT NO AUTO RUNS ARE TO BE STORED HERE!!!!!!!!!
    ///
    ///
    ///
    ///
    ///
    @Override//
    public void runOpMode() throws InterruptedException {
        robot = new AUTOHARD(this, hardwareMap, new Pose2d(-36,-63,Math.toRadians(90.0)));
        waitForStart();
        if (opModeIsActive()){
            robot.justthree();
        }
    }
}

