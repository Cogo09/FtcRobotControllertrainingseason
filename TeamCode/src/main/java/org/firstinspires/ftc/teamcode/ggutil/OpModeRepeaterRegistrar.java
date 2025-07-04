package org.firstinspires.ftc.teamcode.ggutil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.drive.drift.DriftTunerOpMode;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.teleopTracker.TeleOpCopyRunner;
import org.gentrifiedApps.gentrifiedAppsUtil.teleopTracker.TeleOpTrackerOpMode;

public final class OpModeRepeaterRegistrar {
    static String name = "Test2";
    static Driver driver = new Driver("fl", "fr", "bl", "br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
    static boolean isEnabled = true;

    private OpModeRepeaterRegistrar() {
    }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls, OpModeMeta.Flavor flavor) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup("Repeater")
                .setFlavor(flavor)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (!isEnabled) return;
        manager.register(metaForClass(TeleOpCopyRunner.class, OpModeMeta.Flavor.AUTONOMOUS), new TeleOpCopyRunner(name, driver));
        manager.register(metaForClass(TeleOpTrackerOpMode.class, OpModeMeta.Flavor.TELEOP), new TeleOpTrackerOpMode(name, driver));
        manager.register(metaForClass(DriftTunerOpMode.class, OpModeMeta.Flavor.AUTONOMOUS), new DriftTunerOpMode(new Driver("frontLeftMotor", "frontRightMotor", "backLeftMotor", "backRightMotor", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE),3)); //! now has a time param (optional)
    }
}