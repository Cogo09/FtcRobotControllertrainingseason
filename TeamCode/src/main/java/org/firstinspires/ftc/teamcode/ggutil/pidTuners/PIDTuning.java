package org.firstinspires.ftc.teamcode.ggutil.pidTuners;

import com.dacodingbeast.pidtuners.Constants.GravityModelConstants;
import com.dacodingbeast.pidtuners.Constants.PivotSystemConstants;
import com.dacodingbeast.pidtuners.Constants.SlideSystemConstants;
import com.dacodingbeast.pidtuners.HardwareSetup.ArmMotor;
import com.dacodingbeast.pidtuners.HardwareSetup.Hardware;
import com.dacodingbeast.pidtuners.HardwareSetup.SlideMotor;
import com.dacodingbeast.pidtuners.Opmodes.FindPID;
import com.dacodingbeast.pidtuners.Opmodes.FrictionTest;
import com.dacodingbeast.pidtuners.Opmodes.GravityTest;
import com.dacodingbeast.pidtuners.Opmodes.PSODirectionDebugger;
import com.dacodingbeast.pidtuners.Opmodes.SampleOpMode;
import com.dacodingbeast.pidtuners.Opmodes.SlidesTest;
import com.dacodingbeast.pidtuners.Simulators.AngleRange;
import com.dacodingbeast.pidtuners.Simulators.SlideRange;
import com.dacodingbeast.pidtuners.utilities.DataLogger;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;

import CommonUtilities.PIDParams;

public final class PIDTuning {
    public static Double spoolDiameter = 1.5;
    static double frictionRPMs = 416.151979569126;
    static PIDParams pidParamss = new PIDParams(2.971848335360704, 0.9421896872111218, 2.5138363636605914, 0.15749863613320292);
//    static SlideRange slideRange = SlideRange.fromInches(0.0, 12.0);
        static SlideRange slideRange = SlideRange.fromTicks(0.0,700.0);
//        static SlideRange slideRange = SlideRange.fromCM(0.0,30);
    static SlideSystemConstants slideSystemConstants = new SlideSystemConstants(0.42, frictionRPMs);
    public static SlideMotor slideMotor = new SlideMotor.Builder("Slide", DcMotorSimple.Direction.FORWARD,
            Hardware.YellowJacket.RPM435, slideSystemConstants, 1.5, slideRange.asArrayList())
            .pidParams(pidParamss)
            .build();

    static double frictionRPM = 68.0;
    static double inertia = 0.00956980942013831;
    static PIDParams pidParams = new PIDParams(1.3532270829317032, 0.41997561835081976, 0.644510238591845, 0.08260169361314862);
    static AngleRange angleRange = AngleRange.fromDegrees(0.0, 45.0);
    static PivotSystemConstants pivotSystemConstants = new PivotSystemConstants(inertia, frictionRPM, new GravityModelConstants(-0.6820297006437362, 0.2932062495195341, 0.35586130838651726));
    public static ArmMotor armMotor = new ArmMotor.Builder("pivot", DcMotorSimple.Direction.FORWARD, new Hardware.HDHex(Hardware.HDHexGearRatios.GR5_1, Hardware.HDHexGearRatios.GR3_1, Hardware.HDHexGearRatios.GR5_1).getMotorSpecs(), pivotSystemConstants, angleRange.asArrayList())
            .pidParams(pidParams)
            .build();


    public static double gravityMotorPower = 0.5;


    private static OpModeMeta metaForClass(Class<? extends OpMode> cls, String tag) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName() + tag)
                .setGroup("PIDTuners")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    static boolean en = true;

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        DataLogger.getInstance().logDebug(slideMotor.getTargets().get(0).getUnit());
        DataLogger.getInstance().logData("ticksPIn"+ slideMotor.getConversions().getTicksPerInch());
        DataLogger.getInstance().logData("inPTick"+ slideMotor.getConversions().getInchesPerTick());
        if (!en) return;
        manager.register(metaForClass(SlidesTest.class, ""), new SlidesTest(slideMotor));
//        manager.register(metaForClass(PSODirectionDebugger.class, ""), new PSODirectionDebugger(slideMotor, null));
        manager.register(
                metaForClass(FrictionTest.class, "Arm"), new FrictionTest(slideMotor, angleRange, slideRange)
        );
        manager.register(
                metaForClass(SampleOpMode.class, "Arm"), new SampleOpMode(slideMotor)
        );
        manager.register(
                metaForClass(FindPID.class, "Arm"), new FindPID(slideMotor)
        );

        manager.register(
                metaForClass(GravityTest.class, "Arm"), new GravityTest(armMotor, angleRange)
        );
    }
}


