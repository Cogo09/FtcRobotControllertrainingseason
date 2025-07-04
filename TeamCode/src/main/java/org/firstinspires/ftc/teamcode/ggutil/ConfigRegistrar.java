package org.firstinspires.ftc.teamcode.ggutil;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.gentrifiedApps.gentrifiedAppsUtil.config.ConfigCreator;
import org.gentrifiedApps.gentrifiedAppsUtil.config.ConfigMaker;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.kinematics.AutoLevel1DfTuner;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.ServoTesterOpMode;

public final class ConfigRegistrar {
    static ConfigMaker config = new ConfigMaker("pinkbot")
            .addModule(ConfigMaker.ModuleType.EXPANSION_HUB, "Expansion Hub 1")
            .addCamera("Webcam 1","1A8594D0")
            .addMotor("motor", ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.MotorType.RevRoboticsUltraplanetaryHDHexMotor, 0)
            .addDevice("imu", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.REV_INTERNAL_BNO055_IMU, 0)
.addMotor("fl", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.MotorType.RevRoboticsUltraplanetaryHDHexMotor, 3)
.addMotor("fr", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.MotorType.RevRoboticsUltraplanetaryHDHexMotor, 2)
.addMotor("bl", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.MotorType.RevRoboticsUltraplanetaryHDHexMotor, 1)
.addMotor("br", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.MotorType.RevRoboticsUltraplanetaryHDHexMotor, 0)
            .addMotor("pivot", ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.MotorType.goBILDA5202SeriesMotor, 1)
            .addDevice("servo", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.Servo, 0)
            .addDevice("potent", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.AnalogInput, 0)
            .addMotor("enc", ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.MotorType.RevRoboticsUltraplanetaryHDHexMotor, 2)
            .addDevice("touch", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.RevTouchSensor, 0);

    static boolean isEnabled = true;
    private ConfigRegistrar() {
    }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup("Config")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (!isEnabled) return;
        manager.register(metaForClass(ConfigCreator.class), new ConfigCreator(config));
        manager.register(metaForClass(ServoTesterOpMode.class),new ServoTesterOpMode("servo"));
        manager.register(metaForClass(AutoLevel1DfTuner.class),new
                AutoLevel1DfTuner(
                "servo",
                "enc",
                null,
               (double) 90 /8192
        ));

    }
}
