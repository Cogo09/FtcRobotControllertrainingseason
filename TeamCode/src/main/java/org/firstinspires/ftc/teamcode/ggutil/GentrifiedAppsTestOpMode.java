package org.firstinspires.ftc.teamcode.ggutil;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.AnalogEncoder;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.Operand;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.Operation;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.caching.CacheManager;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.callbacks.Timeout;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.drive.DrivePowerCoefficients;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.TimeMachinePair;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.pointClasses.Angle;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.pointClasses.AngleUnit;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.pointClasses.Point;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.pointClasses.Target2D;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.odometer.Odometer;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.speedometer.Speedometer;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.driverAid.DriverAid;
import org.gentrifiedApps.gentrifiedAppsUtil.dataStorage.DataStorage;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.FieldCentricDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.MecanumDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.Button;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.FloatButton;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.GamepadMacro;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.gamepad.GamepadPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.motor.MotorExtensions;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.motor.PIDMotor;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.ServoPlus;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.EncoderSpecs;
import org.gentrifiedApps.gentrifiedAppsUtil.idler.Idler;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;
import org.gentrifiedApps.gentrifiedAppsUtil.motion.controllers.SquIDController;
import org.gentrifiedApps.gentrifiedAppsUtil.motion.profiles.MultiSlewLimiter;
import org.gentrifiedApps.gentrifiedAppsUtil.motion.profiles.SlewRateLimiter;
import org.gentrifiedApps.gentrifiedAppsUtil.motion.profiles.TrapezoidalMotionProfile;
import org.gentrifiedApps.gentrifiedAppsUtil.sensorArray.Sensor;
import org.gentrifiedApps.gentrifiedAppsUtil.sensorArray.SensorArray;

import java.util.List;

enum SMD {
    SLOW_MODE,
    FAST_MODE,
    SUPER_SLOW_MODE
}

enum DA {
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
        GamepadPlus gamepadPlus1 = new GamepadPlus(gamepad1 );
        GamepadPlus gamepadPlus2 = new GamepadPlus(gamepad2);
//        InitMovementController initMovementController = new InitMovementController(gamepadPlus1, gamepadPlus2);
//        InitMovementController initMovementController = new InitMovementController(gamepad1, gamepad2);
        ServoPlus servoPlus = new ServoPlus(this.hardwareMap, "servo");
        Idler idler = new Idler();
//        VoltageTracker voltageTracker = new VoltageTracker(this.hardwareMap);

        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SquIDController squidController = new SquIDController(0.001);

        PIDMotor pidMotor = new PIDMotor(hardwareMap,"motor",0.01,0.0,0.0,0.0);
        pidMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pidMotor.currentReversed();
        pidMotor.reset();
        pidMotor.setTarget(1000);
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
//                .addDriftCorrection(new DrivePowerConstraint(1.0,1.0,1.0,1.0));

        Odometer odo = Odometer.newOdometer(driver);
        odo.addConstraint(new EncoderSpecs(20));


        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        MultiSlewLimiter multiSlew = new MultiSlewLimiter()
                .addLimiter("fl", 0.5)
                .addLimiter("fr", 0.5)
                .addLimiter("bl", 0.5)
                .addLimiter("br", 0.5);


        TrapezoidalMotionProfile trapezoidalMotionProfile = new TrapezoidalMotionProfile(1.0, 0.1);
        trapezoidalMotionProfile.generateProfile(10);

//        AccelerationMotionProfile accelerationMotionProfile = new AccelerationMotionProfile(1.0, 0.1);
//        accelerationMotionProfile.generateProfile(10);

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
                () -> {
                    return false;
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
                () -> {
                    return false;
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
                () -> {
                    return false;
                },
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
                () -> {
                    return false;
                },
                () -> {
                    telemetry.addData("idle", "idle reset");
                }
        );

        func1.runInit();


        DcMotor pivot = hardwareMap.get(DcMotor.class, "pivot");
        MotorExtensions.Companion.resetMotor(pivot);

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
        telemetry.addData("DataStorage", DataStorage.getPose().toString());
        telemetry.addData("DataStorage", DataStorage.getAlliance().toString());
        DataStorage.initDataStore();
        DataStorage.readDataStore();
        telemetry.addData("DataStorage", DataStorage.getPose().toString());
        telemetry.addData("DataStorage", DataStorage.getAlliance().toString());
        telemetry.update();

        Speedometer speedometer = Speedometer.newSpeedometer(driver,EncoderSpecs.ticksPerIn(20));

        Scribe.getInstance().startLogger("GentrifiedAppsTestOpMode");
        waitForStart();
        Timeout timeout = new Timeout(5, () -> {
            return gamepadPlus1.buttonJustReleased(Button.DPAD_DOWN);
        });
        timeout.start();
        trapezoidalMotionProfile.start();
//        accelerationMotionProfile.start();
        GamepadMacro macro = new GamepadMacro(List.of((Button.L1),(Button.L1)), () -> {
            timeout.start();
            timeout.update();
        });
//        GamepadMacro macro2 = new GamepadMacro(new MacroBuilder().buttonPress(Button.CROSS).buttonPress(Button.CROSS).build(), () -> {
//            timeout.start();
//            timeout.update();
//        });

        SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.5);
//        OnlyUpSlewRateLimiter slewRateLimiter = new OnlyUpSlewRateLimiter(0.5);
//        pidMotor.currentReversed();
//        pidMotor.setTarget(1000);

        loopTimeController.setLoopSavingCache(hardwareMap);
        while (opModeIsActive()) {
            speedometer.update();
            speedometer.telemetry(telemetry);
            odo.update();
            odo.telemetry(telemetry);
            macro.update(gamepadPlus1);
            if (gamepadPlus1.buttonPressed(Button.DPAD_RIGHT)) {
                servoPlus.setPosition(90);
                driverAid.setDriverAidFunction(func1);
            } else if (gamepadPlus1.buttonPressed(Button.DPAD_UP)) {
                func2.runInit();
            } else if (gamepadPlus1.buttonPressed(Button.DPAD_LEFT)) {
                func3.runInit();
                DataStorage.setPose(new Target2D(80, 111, 2));
                DataStorage.writeDataStore();
            } else if (gamepadPlus1.buttonJustPressed(Button.DPAD_DOWN)) {
                servoPlus.setPosition(0);
                driverAid.idle(DA.IDLE);
                DataStorage.setPose(new Target2D(90.0, 10.0, 20.0));
            }
            driverAid.update();

            telemetry.addData("aEncoder", aEncoder.getCurrentPosition());
            sensorArray.readAllLoopSaving();
            timeout.update();
            telemetry.addData("timeout", timeout.isTimedOut());

//            motor.setPower(squidController.calculate(1000,motor.getCurrentPosition()));
pidMotor.setPIDPower();
telemetry.addData("pidMotor", pidMotor.getCurrentPosition());
            motor.setPower(slewRateLimiter.calculate(gamepadPlus1.readFloat(FloatButton.RIGHT_TRIGGER)));
//            motor.setPower(slewRateLimiter.calculate(gamepadPlus1.readFloat(FloatButton.RIGHT_TRIGGER)));
//            motor.setPower(accelerationMotionProfile.getVelocity());
//            double trap = trapezoidalMotionProfile.getVelocity();
//            motor.setPower(trap);
//            telemetry.addData("trapezoidalMotionProfile", trap);

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
//            DrivePowerCoefficients powerCoefficients = FieldCentricDriver.driveFieldCentric(gamepadPlus1.readFloat(FloatButton.LEFT_X), gamepadPlus1.readFloat(FloatButton.LEFT_Y), gamepadPlus1.readFloat(FloatButton.RIGHT_X), new Angle(imu.getRobotYawPitchRollAngles().getYaw(), AngleUnit.DEGREES));
            DrivePowerCoefficients powerCoefficients = MecanumDriver.driveMecanum(-gamepadPlus1.readFloat(FloatButton.LEFT_X),gamepadPlus1.readFloat(FloatButton.LEFT_Y),-gamepadPlus1.readFloat(FloatButton.RIGHT_X));

//            if (!powerCoefficients.notZero()){
//                // has no powers to move
//                teleOpCorrector.updateOrientation();
//            }else if (gamepadPlus1.readFloat(FloatButton.RIGHT_X) == 0.0){
//                powerCoefficients = teleOpCorrector.correctByAngle(powerCoefficients);
//            }
//            driver.setWheelPower(Driver.applyDriftCorrection(powerCoefficients));

            driver.setWheelPower(powerCoefficients);
//            driver.setWheelPower(
//                    new DrivePowerCoefficients(
//                            multiSlew.calculate("fl",powerCoefficients.getFrontLeft()),
//                            multiSlew.calculate("fr",powerCoefficients.getFrontRight()),
//                            multiSlew.calculate("bl",powerCoefficients.getBackLeft()),
//                            multiSlew.calculate("br",powerCoefficients.getBackRight())));
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
            gamepadPlus1.sync();
            gamepadPlus2.sync();
            telemetry.update();
        }
    }
}
