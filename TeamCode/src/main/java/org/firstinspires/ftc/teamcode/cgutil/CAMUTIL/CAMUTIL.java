package org.firstinspires.ftc.teamcode.cgutil.CAMUTIL;

import com.qualcomm.robotcore.hardware.Servo;

public class CAMUTIL {
    public static void setpose(Servo servo, double degrees) {
        final double degreemult = 0.00555555554;
        servo.setPosition(degreemult * degrees);
    }

    public static boolean inRange(double val1, double val2, double target) {
        return (val1 < target && val2 > target) || (val2 < target && val1 > target);
    }
}
