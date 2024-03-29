package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwarePerseverence;

import java.util.List;

@Autonomous(name = "RedAutoRight", group = "Bot")
public class RedAutoRight extends LinearOpMode {
    // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
    double encoderMeasurementCovariance = 0.8;
    HardwarePerseverence robot = new HardwarePerseverence();
    private final ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    Orientation angles;
    Camera camera = new Camera(robot, imu, hardwareMap);
    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CENTIMETERS = 10.16;     // For figuring circumference
    static final double COUNTS_PER_CENTIMETER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CENTIMETERS * 3.1415);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 0.8;
    static final double DIST_PER_REV = (10 * Math.PI) / COUNTS_PER_MOTOR_REV;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "Afz+Zrv/////AAABmSYDPqm+9Eeal4Qr3dPA93OKxmHLJOElO/flBfFW0x0sVdkLfIA4pm1EPqtd8fnTd0N0fKtVH0+hBxmglqS/Xn7S7Ahjpfd6sHeA4db+jCCf+lKvrt5LCuHp14NsBsjs4QB90u4hp/qGKQOyP4PVZ1KLC5gDu0VCP5vu4+M6505T4vAmojax18+tNGWInbVpUPqJuQPHregDhjsv7/6P/RLQMMK0zWW4zHCyiatZZX3NzGrrHvMh3uPE3mN/E0ucG/guL6Pp16PpLevoUXoz6IqqmWXcZUMo/IkRKdppCspWJbRfSngK5B2+MNzbM3tedVzFGaey/HApKBMxuqhO5uY+nWSud0GLd/SpJ9EmMRdl";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    double openEscape = 0;
    double closedEscape = .3;
    double finalOpenEscape = .05;
    double finalClosedEscape = .255;


    public double subtractAngle(double angleA,
                                double angleB) {
        double result;
        result = angleA - angleB;
        if (result > 180) {
            result = result - 360;
        }
        return result;
    }

    public void waitMilis(double timeOutMs) {

        runtime.reset();
        while (runtime.milliseconds() < timeOutMs) ;
    }

    public void autoPilot(double heading,
                          double pose,
                          double distance,
                          double power,
                          double timeoutS) {

        double currentHeading;
        double drvPower = power;
        double adjPower;
        double currentDistance;
        double driveAngle;
        double headingRadians;
        double poseDegrees;
        double distX = 0;
        double distY = 0;
        double dY;
        double dX;
        double oldLeft = robot.leftDrive.getCurrentPosition();
        double oldRight = robot.rightDrive.getCurrentPosition();
        double oldBackLeft = robot.leftBackDrive.getCurrentPosition();
        double oldBackRight = robot.rightBackDrive.getCurrentPosition();
        double newLeft = 0;
        double newRight = 0;
        double newBackLeft = 0;
        double newBackRight = 0;
        runtime.reset();
        while ((runtime.seconds() < timeoutS)) {
            currentHeading = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            headingRadians = ((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle) + (2.5 * Math.PI)) % (2 * Math.PI);
            poseDegrees = ((pose - 3.1416 / 2) % (2 * 3.1416)) * (360 / (2 * 3.1416));
            if (poseDegrees > 180) {
                poseDegrees -= 360;
            }
            if (poseDegrees < -180) {
                poseDegrees += 360;
            }
            driveAngle = subtractAngle(poseDegrees, currentHeading);
            adjPower = subtractAngle(poseDegrees, currentHeading) / 120;
            newLeft = robot.leftDrive.getCurrentPosition();
            newRight = robot.rightDrive.getCurrentPosition();
            newBackRight = robot.rightBackDrive.getCurrentPosition();
            newBackLeft = robot.leftBackDrive.getCurrentPosition();

            dX = ((((newLeft - oldLeft) - (newBackLeft - oldBackLeft)
                    - (newRight - oldRight) + (newBackRight - oldBackRight)) * Math.sin(Math.PI / 2)) / 4.0) * DIST_PER_REV;
            dY = (((newLeft - oldLeft) + (newBackLeft - oldBackLeft)
                    + (newRight - oldRight) + (newBackRight - oldBackRight)) / 4.0) * DIST_PER_REV;
            distX += (Math.sin(headingRadians) * dX + Math.cos(headingRadians) * dY);
            distY += (Math.sin(headingRadians) * dY + Math.cos(headingRadians) * dX);
            oldLeft = newLeft;
            oldRight = newRight;
            oldBackLeft = newBackLeft;
            oldBackRight = newBackRight;

            currentDistance = Math.sqrt(distX * distX + distY * distY);
            if (currentDistance > distance) {
                robot.leftDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
                return;
            }

            driveAngle = ((-currentHeading / 180) * 3.1416) + (1 / 2 * 3.1416) + heading;
//            driveAngle = 0.5*Math.PI;
            adjPower = subtractAngle(poseDegrees, currentHeading) / 120;

            double robotAngle = driveAngle - Math.PI / 4;
            final double v1 = power * Math.cos(robotAngle) - adjPower;
            final double v2 = power * Math.sin(robotAngle) + adjPower;
            final double v3 = power * Math.sin(robotAngle) - adjPower;
            final double v4 = power * Math.cos(robotAngle) + adjPower;


            robot.leftDrive.setPower(v1);
            robot.rightDrive.setPower(v2);
            robot.leftBackDrive.setPower(v3);
            robot.rightBackDrive.setPower(v4);
        }
    }


    public void whileAutoPilot(double heading,
                               double pose,
                               double power) {
        double currentHeading;
        double adjPower;
        double driveAngle;
        double headingRadians;
        double poseDegrees;

        currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        poseDegrees = ((pose - 3.1416 / 2) % (2 * 3.1416)) * (360 / (2 * 3.1416));
        if (poseDegrees > 180) {
            poseDegrees -= 360;
        }
        if (poseDegrees < -180) {
            poseDegrees += 360;
        }
        adjPower = subtractAngle(poseDegrees, currentHeading) / 120;
        driveAngle = ((-currentHeading / 180) * 3.1416) + (1 / 2 * 3.1416) + heading;
//            driveAngle = 0.5*Math.PI;
//            adjPower = 0;
        double robotAngle = driveAngle - Math.PI / 4;
        final double v1 = power * Math.cos(robotAngle) - adjPower;
        final double v2 = power * Math.sin(robotAngle) + adjPower;
        final double v3 = power * Math.sin(robotAngle) - adjPower;
        final double v4 = power * Math.cos(robotAngle) + adjPower;
        robot.leftDrive.setPower(v1);
        robot.rightDrive.setPower(v2);
        robot.leftBackDrive.setPower(v3);
        robot.rightBackDrive.setPower(v4);
        return;
    }

    public void gyroTurn(double heading,
                         double timeoutS) {
        double currentHeading;
        double power;
//        double p = 0;
//        double i = 0;
//        double d = 0;
//        double pK = .05;
//        double iK = .5;
//        double dK = .05;
//        double oldHeading = 0;
        runtime.reset();
        while (runtime.seconds() < timeoutS) {
            currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//            p = (currentHeading - heading) * pK;
//            i = p * iK + i;
//            d = (oldHeading - currentHeading) * dK;
//            power = p+i+d;
            power = (currentHeading - heading * (180 / Math.PI)) / 50;
            if (power < 0) {
                power = -Math.max(0.1, Math.abs(power));
            } else {
                power = Math.max(0.1, Math.abs(power));
            }
            if (Math.abs(currentHeading - heading) < 2.0) {
                return;
            }

            robot.leftDrive.setPower(-power);
            robot.leftBackDrive.setPower(-power);
            robot.rightDrive.setPower(power);
            robot.rightBackDrive.setPower(power);
        }
    }

    public void brake() {
        robot.leftDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        return;
    }

    public void autoPowerShot(double flyWheelPower,
                              double lookingGlassPower) {
        double openEscape = 0;
        double closedEscape = .2;
        double time;
        robot.flyWheel.setPower(flyWheelPower);
        robot.lookingGlass.setPower(lookingGlassPower);
        waitMilis(500);
        autoPilot(0.0, 1.57, 8, .65, 10);
        robot.escapeServo.setPosition(openEscape);
        robot.finalEscapeServo.setPosition(openEscape);
        time = runtime.milliseconds();
        while (runtime.milliseconds() - time < 2500) {
            whileAutoPilot(0.0, 1.57, .24);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.45f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void runOpMode() {
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        robot.choker.setPosition(.4125);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0 / 9.0);
        }
        //reset the encoders to 0
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //run using the encoders
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitMilis(100);
        telemetry.addData("Robot:", "Ready For Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //Universal Start

            autoPilot(1.57, 2.7, 20, .8, 6);
            autoPilot(5.93, 2.7, 5, .7, 6);
            brake();
            waitMilis(1000);
            boolean quad = false;
            boolean single = false;
            boolean none = false;
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            //telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel() == "Quad") { // Quad
                    quad = true;
                    single = false;
                    none = false;
                } else if (recognition.getLabel() == "Single") { // Single
                    single = true;
                    quad = false;
                    none = false;
                } else { /** None **/
                    none = true;
                    single = false;
                    quad = false;
                }
            }

                if (quad) {
                    telemetry.addData("Working?", "Quad");
                    telemetry.update();
                    autoPilot(1.57, 1.57, 170, .7, 6);
                    waitMilis(200);
                    while (robot.rightBottomColor.red() < 500) {
                        whileAutoPilot(1.57, 1.57, .2);
                    }
                    brake();
                    autoPilot(3.14,4.71,20,.6,6);
                    robot.arm.setPower(1);
                    autoPilot(0,4.71,13,.6,6);
                    waitMilis(200);
                    robot.choker.setPosition(0);
                    robot.arm.setPower(0);
                    waitMilis(500);
                    robot.arm.setPower(-1);
                    autoPilot(4.71,4.71,40,.6,6);
                    robot.arm.setPower(0);


                } else if (single) {
                    telemetry.addData("Working?", "Single");
                    telemetry.update();
                    autoPilot(1.57, 1.57, 170, .7, 6);
                    waitMilis(200);
                    while (robot.rightBottomColor.red() < 500) {
                        whileAutoPilot(1.57, 1.57, .2);
                    }
                    brake();
                    autoPilot(3.14, 0, 20, .5, 6);
                    robot.arm.setPower(1);
                    autoPilot(0, 0, 15, .5, 6);
                    robot.choker.setPosition(0);
                    waitMilis(300);
                    robot.arm.setPower(-1);
                    waitMilis(200);
                    robot.arm.setPower(0);
                    autoPilot(4.71, 0, 60, .5, 5);
                    stop();


                } else {

                    telemetry.addData("Working?", "None");
                    telemetry.update();
                    autoPilot(1.57, 1.57, 170, .7, 6);
                    waitMilis(200);
                    robot.arm.setPower(1);
                    while (robot.rightBottomColor.red() < 500) {
                        whileAutoPilot(1.57, 1.57, .2);
                    }
                    brake();
                    autoPilot(1.57,1.57,10,.5,4);
                    robot.choker.setPosition(0);
                    waitMilis(300);
                    robot.arm.setPower(-1);
                    waitMilis(1000);
                    robot.arm.setPower(0);
                    waitMilis(10000);
                    autoPilot(3.14,1.57,70,.7,6);
                    autoPilot(4.71, 1.57, 40, .5, 5);
                    stop();
                }


                stop();


            if (tfod != null) {
                tfod.shutdown();
            }
            waitMilis(5000);
            stop();
        }
    }
}
