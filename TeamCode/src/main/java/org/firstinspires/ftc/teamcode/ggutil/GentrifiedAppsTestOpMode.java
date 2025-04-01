package org.firstinspires.ftc.teamcode.ggutil;


import android.util.Pair;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.gentrifiedApps.gentrifiedAppsUtil.classExtenders.gamepad.Button;
import org.gentrifiedApps.gentrifiedAppsUtil.classExtenders.gamepad.FloatButton;
import org.gentrifiedApps.gentrifiedAppsUtil.classExtenders.gamepad.GamepadPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.classExtenders.servo.ServoPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.Timeout;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.AnalogEncoder;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.Operand;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.Operation;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.DrivePowerCoefficients;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.FieldCentricDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.Angle;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.AngleUnit;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.localizers.tracking.MecanumLocalizer;
import org.gentrifiedApps.gentrifiedAppsUtil.idler.Idler;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;
import org.gentrifiedApps.gentrifiedAppsUtil.motionProfiles.SlewRateLimiter;
import org.gentrifiedApps.gentrifiedAppsUtil.sensorArray.Sensor;
import org.gentrifiedApps.gentrifiedAppsUtil.sensorArray.SensorArray;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

enum SMD {
    SLOW_MODE,
    FAST_MODE,
    SUPER_SLOW_MODE
}

enum DA{
    DRIVE,
    TURN,
    TEST,
    IDLE
}
@TeleOp
public class GentrifiedAppsTestOpMode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        LoopTimeController loopTimeController = new LoopTimeController();
        GamepadPlus gamepadPlus1 = new GamepadPlus(gamepad1, false);
        GamepadPlus gamepadPlus2 = new GamepadPlus(gamepad2, false);
//        InitMovementController initMovementController = new InitMovementController(gamepadPlus1, gamepadPlus2);
//        InitMovementController initMovementController = new InitMovementController(gamepad1, gamepad2);
        ServoPlus servoPlus = new ServoPlus(this.hardwareMap, "servo");
        Idler idler = new Idler(this);
//        VoltageTracker voltageTracker = new VoltageTracker(this.hardwareMap);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

//    TwoWheelLocalizer localizer = new TwoWheelLocalizer(
//            hardwareMap,
//            new Target2D(0,0,new Angle(0,AngleUnit.RADIANS)),
//            new EncoderStorage(new Encoder(EncoderSpecsBuilder.INSTANCE.goBildaSwingArm(), "fl", DcMotorSimple.Direction.FORWARD,0.0,hardwareMap),null
//            ,new Encoder(EncoderSpecsBuilder.INSTANCE.goBildaSwingArm(), "fr", DcMotorSimple.Direction.FORWARD,0.0,hardwareMap)),
//            new IMUParams("imu", new IMU.Parameters(orientationOnRobot))
//    );
        Driver driver = new Driver(this, "fl", "fr", "bl", "br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);

        MecanumLocalizer localizer = new MecanumLocalizer(driver, 37.74, 16);
        driver.setLocalizer(localizer);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();


        DriverAid driverAid = new DriverAid<>(DA.class);
        DriverAid.DAFunc func1 = new DriverAid.DAFunc(
                driverAid,
                DA.DRIVE,
                () -> {
                    telemetry.addData("func1", "drive init");
                },
                () -> {
                    telemetry.addData("func1", "drive constant");
                },
                ()->{return false;
                    },
                () -> {
                    telemetry.addData("func1", "drive reset");
                }
        );

        DriverAid.DAFunc func2 = new DriverAid.DAFunc(
                driverAid,
                DA.TURN,
                () -> {
                    telemetry.addData("func2", "turn init");
                },
                () -> {
                    telemetry.addData("func2", "turn constant");
                },
                ()->{return false;
                },
                () -> {
                    telemetry.addData("func2", "turn reset");
                }
        );

        DriverAid.DAFunc func3 = new DriverAid.DAFunc(
                driverAid,
                DA.TEST,
                () -> {
                    telemetry.addData("func3", "test init");
                },
                () -> {
                    telemetry.addData("func3", "test constant");
                },
                ()->{return false;},
                () -> {
                    telemetry.addData("func3", "test reset");
                }
        );
        DriverAid.DAFunc idle = new DriverAid.DAFunc(
                driverAid,
                DA.IDLE,
                () -> {
                    telemetry.addData("idle", "idle init");
                },
                () -> {
                    telemetry.addData("idle", "idle constant");
                },
                ()->{return false;},
                () -> {
                    telemetry.addData("idle", "idle reset");
                }
        );

        func1.runInit();

//        SlowModeManager slowModeManager = new SlowModeManager(gamepadPlus1);// test with just 1 and default button

//        HashMap<Enum<?>, SlowModeMulti> slowModeMap = new HashMap<>();
//        slowModeMap.put(SMD.SLOW_MODE, SlowModeMulti.basic());
//        slowModeMap.put(SMD.FAST_MODE, new SlowModeMulti(SlowMode.one(), Button.B));
//        slowModeMap.put(SMD.SUPER_SLOW_MODE, new SlowModeMulti(new SlowMode(3.0), Button.X));
//
//        SlowModeManager slowModeManager = new SlowModeManager(slowModeMap, gamepadPlus1);

        AnalogEncoder aEncoder = new AnalogEncoder(hardwareMap, "potent", 0.0, List.of(new Operation(Operand.MULTIPLY, 81.8)));

        SensorArray sensorArray = new SensorArray()
                .addSensor(Sensor.touchSensor(hardwareMap, "touch"))
                .addSensor(Sensor.analogEncoder(aEncoder));
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        driver.update();

        Scribe.getInstance().startLogger("GentrifiedAppsTestOpMode");
        waitForStart();
        Timeout timeout = new Timeout(5, () -> {
            return gamepadPlus1.buttonJustReleased(Button.DPAD_DOWN);
        });
        timeout.start();

        SlewRateLimiter flRateLimiter = new SlewRateLimiter(0.5);

        loopTimeController.setLoopSavingCache(hardwareMap);
        while (opModeIsActive()) {
            if (gamepadPlus1.buttonPressed(Button.DPAD_RIGHT)) {
                driverAid.setDriverAidFunction(func1);
            }else if (gamepadPlus1.buttonPressed(Button.DPAD_UP)){
                func2.runInit();
            }else if (gamepadPlus1.buttonPressed(Button.DPAD_LEFT)){
                func3.runInit();
            }else if (gamepadPlus1.buttonPressed(Button.DPAD_DOWN)){
                driverAid.idle(DA.IDLE);
            }
            driverAid.update();

            telemetry.addData("aEncoder", aEncoder.getCurrentPosition());
            sensorArray.readAllLoopSaving();
            timeout.update();
            telemetry.addData("timeout", timeout.isTimedOut());

            if (gamepadPlus1.readFloat(FloatButton.RIGHT_TRIGGER)>0){
                driver.setWheelPower(new DrivePowerCoefficients(flRateLimiter.calculate(gamepadPlus1.readFloat(FloatButton.RIGHT_TRIGGER)),0.0,0.0,0.0));
            }
//            loopTimeController.every(1,()->{});

//            voltageTracker.update();
//            voltageTracker.telemetry(telemetry);
//            initMovementController.checkHasMovedOnInit();

//            if (initMovementController.hasMovedOnInit()) {
//                servoPlus.setPosition(180);
//            }

//            if (initMovementController.hasMovedOnInit()) {
//            if (gamepadPlus1.buttonJustReleased(Button.A)) {
//                idler.safeIdle(5, () -> {
//                    telemetry.addData("idler", "idling");
//                    telemetry.update();
//                });
//            }
//            }
            

            telemetry.addData("imu", imu.getRobotYawPitchRollAngles().getYaw());
            DrivePowerCoefficients powerCoefficients = FieldCentricDriver.driveFieldCentric(gamepadPlus1.readFloat(FloatButton.LEFT_X), gamepadPlus1.readFloat(FloatButton.LEFT_Y), gamepadPlus1.readFloat(FloatButton.RIGHT_X), new Angle(imu.getRobotYawPitchRollAngles().getYaw(), AngleUnit.DEGREES));
//            DrivePowerCoefficients powerCoefficients = MecanumDriver.driveMecanum(gamepadPlus1.readFloat(FloatButton.LEFT_X),gamepadPlus1.readFloat(FloatButton.LEFT_Y),gamepadPlus1.readFloat(FloatButton.RIGHT_X));

//            if (!powerCoefficients.notZero()){
//                // has no powers to move
//                teleOpCorrector.updateOrientation();
//            }else if (gamepadPlus1.readFloat(FloatButton.RIGHT_X) == 0.0){
//                powerCoefficients = teleOpCorrector.correctByAngle(powerCoefficients);
//            }
            driver.setWheelPower(powerCoefficients);
//            driver.setWheelPower(powerCoefficients.applySlowMode(slowModeManager));
//            driver.setWheelPower(new DrivePowerCoefficients(
//                   slowModeManager.apply(powerCoefficients.getFrontLeft()),
//                    slowModeManager.apply(powerCoefficients.getFrontRight()),
//                    slowModeManager.apply(powerCoefficients.getBackLeft()),
//                    slowModeManager.apply(powerCoefficients.getBackRight()))
//            );

            loopTimeController.update();

//            telemetry.addData("hasMovedOnInit", initMovementController.hasMovedOnInit());
            loopTimeController.telemetry(telemetry);
//            slowModeManager.telemetry(telemetry);
            sensorArray.allTelemetry(telemetry);
            driver.update();
            driver.getLocalizer().testEncoderDirection(telemetry);
            gamepadPlus1.sync();
            gamepadPlus2.sync();
        }
    }

}
