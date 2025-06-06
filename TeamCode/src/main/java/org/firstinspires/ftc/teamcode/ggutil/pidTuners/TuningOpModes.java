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
import com.dacodingbeast.pidtuners.Simulators.AngleRange;
import com.dacodingbeast.pidtuners.Simulators.SlideRange;
import com.dacodingbeast.pidtuners.utilities.DataLogger;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import CommonUtilities.PIDParams;


import com.dacodingbeast.pidtuners.Constants.GravityModelConstants;
import com.dacodingbeast.pidtuners.Constants.PivotSystemConstants;
import com.dacodingbeast.pidtuners.HardwareSetup.ArmMotor;
import com.dacodingbeast.pidtuners.HardwareSetup.Hardware;
import com.dacodingbeast.pidtuners.Simulators.AngleRange;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import CommonUtilities.PIDParams;

public final class TuningOpModes {
//    public static Double spoolDiameter = 1.0;
//    static double frictionRPM = 0.0;
//    static PIDParams pidParams = new PIDParams(0.0, 0.0, 0.0, 0.0);
//    static SlideRange slideRange = new SlideRange(0.0, 38.0);
//    static SlideSystemConstants slideSystemConstants = new SlideSystemConstants(0.0, frictionRPM);
//    public static SlideMotor slideMotor = new SlideMotor("Slide", DcMotorSimple.Direction.FORWARD,
//            Hardware.YellowJacket.RPM223,
//            spoolDiameter, slideSystemConstants, 1.0, pidParams, slideRange.asArrayList());

    static double frictionRPM = 0.0;
    static double inertia = 0.0;
    static PIDParams pidParams = new PIDParams(0.0, 0.0, 0.0, 0.0);
    static AngleRange angleRange = AngleRange.fromDegrees(0.0,90.0);
    static PivotSystemConstants pivotSystemConstants = new PivotSystemConstants(inertia, frictionRPM, new GravityModelConstants(0.0,0.0,0.0));
    public static ArmMotor armMotor = new ArmMotor.Builder("pivot", DcMotorSimple.Direction.REVERSE,Hardware.YellowJacket.RPM435,pivotSystemConstants,angleRange.asArrayList())
            .pidParams(pidParams)
            .build();


    public static double start = 0.0;
    public static double gravityMotorPower = 0.0;


    private static OpModeMeta metaForClass(Class<? extends OpMode> cls, String tag) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName() + tag)
                .setGroup("PIDTuners")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }
    static boolean en = false;
    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (!en) return;
        manager.register(metaForClass(PSODirectionDebugger.class, ""), new PSODirectionDebugger(null, armMotor));
        manager.register(
                metaForClass(FrictionTest.class, "Arm"), new FrictionTest(armMotor)
        );
        manager.register(
                metaForClass(SampleOpMode.class, "Arm"), new SampleOpMode(armMotor)
        );
        manager.register(
                metaForClass(FindPID.class, "Arm"), new FindPID(armMotor)
        );

        manager.register(
                    metaForClass(GravityTest.class, "Arm"), new GravityTest(armMotor)
            );
    }
}


