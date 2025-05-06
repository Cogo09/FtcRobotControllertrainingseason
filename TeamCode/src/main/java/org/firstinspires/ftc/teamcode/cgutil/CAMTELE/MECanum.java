package org.firstinspires.ftc.teamcode.cgutil.CAMTELE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cgutil.CAMHARDWARE.HARDConfig;

//import org.firstinspires.ftc.teamcode.Hardware.HARDconfig;

@TeleOp
public class MECanum extends LinearOpMode {
    HARDConfig robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HARDConfig(this, hardwareMap, false);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.dobulk();
        }
    }
}