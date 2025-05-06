package org.firstinspires.ftc.teamcode.cgutil.CAMUTIL;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CAMDUALENCODER {
    DcMotor encoder = null;
    DcMotor encoder1 = null;

    public CAMDUALENCODER(DcMotor enc, DcMotor enc1) {
        encoder = enc;
        encoder1 = enc1;
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getmost() {

        if (encoder1.getCurrentPosition() > encoder.getCurrentPosition() + 80) {
            return encoder1.getCurrentPosition();
        } else {
            return encoder.getCurrentPosition() + 80;
        }
    }

    public double getAverage() {

        return (-encoder1.getCurrentPosition() + encoder.getCurrentPosition()) / 2.0;

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("encoder", encoder.getCurrentPosition() + 80);
        telemetry.addData("encoder1", encoder1.getCurrentPosition());
    }
}

