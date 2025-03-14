package org.firstinspires.ftc.teamcode.ggutil;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

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
import org.gentrifiedApps.gentrifiedAppsUtil.drive.MecanumDriver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.TeleOpCorrector;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.Encoder;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generators.EncoderSpecsBuilder;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.EncoderSpecs;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.EncoderStorage;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.PIDController;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.Angle;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.AngleUnit;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Heatseeker;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.generics.pointClasses.Target2D;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.localizers.tracking.IMUParams;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.localizers.tracking.TwoWheelLocalizer;
import org.gentrifiedApps.gentrifiedAppsUtil.idler.Idler;
import org.gentrifiedApps.gentrifiedAppsUtil.drive.FieldCentricDriver.Companion.*;
import org.gentrifiedApps.gentrifiedAppsUtil.initMovement.InitMovementController;
import org.gentrifiedApps.gentrifiedAppsUtil.looptime.LoopTimeController;
import org.gentrifiedApps.gentrifiedAppsUtil.sensorArray.Sensor;
import org.gentrifiedApps.gentrifiedAppsUtil.sensorArray.SensorArray;
import org.gentrifiedApps.gentrifiedAppsUtil.sensorArray.SensorType;

import java.lang.annotation.Target;
import java.util.List;

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
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

//    TwoWheelLocalizer localizer = new TwoWheelLocalizer(
//            hardwareMap,
//            new Target2D(0,0,new Angle(0,AngleUnit.RADIANS)),
//            new EncoderStorage(new Encoder(EncoderSpecsBuilder.INSTANCE.goBildaSwingArm(), "fl", DcMotorSimple.Direction.FORWARD,0.0,hardwareMap),null
//            ,new Encoder(EncoderSpecsBuilder.INSTANCE.goBildaSwingArm(), "fr", DcMotorSimple.Direction.FORWARD,0.0,hardwareMap)),
//            new IMUParams("imu", new IMU.Parameters(orientationOnRobot))
//    );
        Driver driver = new Driver(this,"fl","fr","bl","br", DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        AnalogEncoder aEncoder = new AnalogEncoder(hardwareMap,"potent", 0.0,List.of(new Operation(Operand.MULTIPLY,81.8)));

        SensorArray sensorArray = new SensorArray(hardwareMap);
        sensorArray.addSensor(new Sensor("touch", SensorType.TOUCH,1));
        sensorArray.addSensor(new Sensor(aEncoder,1));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        driver.update();

        Scribe.getInstance().startLogger("CoolTag");
        waitForStart();
        Timeout timeout = new Timeout(5,()->{return gamepadPlus1.buttonJustReleased(Button.DPAD_DOWN);});
        timeout.start();
        while (opModeIsActive()) {
            telemetry.addData("aEncoder",aEncoder.getCurrentPosition());
            sensorArray.autoLoop(loopTimeController.getLoops());
            sensorArray.allTelemetry(telemetry);
            timeout.update();
            telemetry.addData("timeout",timeout.isTimedOut());
//            voltageTracker.update();
//            voltageTracker.telemetry(telemetry);
//            initMovementController.checkHasMovedOnInit();

//            if (initMovementController.hasMovedOnInit()) {
//                servoPlus.setPosition(180);
//            }

//            if (initMovementController.hasMovedOnInit()) {
                if (gamepadPlus1.buttonJustReleased(Button.A)) {
                    idler.safeIdle(5, () -> {
                        telemetry.addData("idler", "idling");
                        telemetry.update();
                    });
                }
//            }

            telemetry.addData("imu",imu.getRobotYawPitchRollAngles().getYaw());
//            DrivePowerCoefficients powerCoefficients = FieldCentricDriver.driveFieldCentric(gamepadPlus1.readFloat(FloatButton.LEFT_X),gamepadPlus1.readFloat(FloatButton.LEFT_Y),gamepadPlus1.readFloat(FloatButton.RIGHT_X),imu.getRobotYawPitchRollAngles().getYaw());
            DrivePowerCoefficients powerCoefficients = MecanumDriver.driveMecanum(gamepadPlus1.readFloat(FloatButton.LEFT_X),gamepadPlus1.readFloat(FloatButton.LEFT_Y),gamepadPlus1.readFloat(FloatButton.RIGHT_X));

//            if (!powerCoefficients.notZero()){
//                // has no powers to move
//                teleOpCorrector.updateOrientation();
//            }else if (gamepadPlus1.readFloat(FloatButton.RIGHT_X) == 0.0){
//                powerCoefficients = teleOpCorrector.correctByAngle(powerCoefficients);
//            }

            driver.setWheelPower(powerCoefficients);

            loopTimeController.update();
            gamepadPlus1.sync();
            gamepadPlus2.sync();

//            telemetry.addData("hasMovedOnInit", initMovementController.hasMovedOnInit());
            loopTimeController.telemetry(telemetry);
            driver.update();
            telemetry.update();
        }
    }

}
