package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwarePerseverence;

@Autonomous(name = "Auto", group = "Pushbot")
public class RedAuto extends LinearOpMode {
    Transform2d cameraToRobot = new Transform2d();
    // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
    double encoderMeasurementCovariance = 0.8;
    // Set to the starting pose of the robot
    Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());

    T265Camera slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
//slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin
    HardwarePerseverence robot = new HardwarePerseverence();
    private final ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    Orientation angles;
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CENTIMETERS = 10.16;     // For figuring circumference
    static final double COUNTS_PER_CENTIMETER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CENTIMETERS * 3.1415);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 0.8;
    static final double DIST_PER_REV = (10 * Math.PI) / COUNTS_PER_MOTOR_REV;

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

    public void brake() {
        robot.leftDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        return;
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
        waitMilis(50);
        waitForStart();
        while (opModeIsActive()) {
            // go to line
            autoPilot(1.57, 1.57, 120, .4, 10);
            while (((double) robot.bottomColor.red() < 40) && ((double) robot.bottomColor.blue() < 40) && ((double) robot.bottomColor.green() < 40)) {
                whileAutoPilot(1.57, 1.57, .4);
            }
            runtime.reset();
            autoPilot(1.1, 1.57, 117.5, 0.6 - runtime.seconds() / 7, 10);
            //drop rings
            waitMilis(200);
            autoPilot(4.71, 1.57, 100, .6, 10);
            autoPilot(3.14, 1.57, 105, .7, 10);
            waitMilis(10);
            //grab wobble
            autoPilot(4.71, 4.71, 88, .7, 10);
//            runtime.reset();
//            while (((double) robot.bottomColor.red() < 28) && (runtime.seconds() < 4)) {
//                whileAutoPilot(0, 4.71, .2);
//            }
//            autoPilot(3.14, 4.71, 37, .7, 10);
            autoPilot(4.71, 4.71, 4, .7, 10);
            waitMilis(200);
            waitMilis(1000);
            autoPilot(1.57, 0, 100, .7, 10);
            while (((double) robot.bottomColor.red() < 40) && ((double) robot.bottomColor.blue() < 40) && ((double) robot.bottomColor.green() < 40)) {
                whileAutoPilot(1.57, 0, .4);
            }
            autoPilot(0, 0, 85, 0.5, 10);
            waitMilis(600);
            autoPilot(3.14, 0, 75, 0.7, 10);
//            while (((double) robot.bottomColor.red() < 40) & ((double) robot.bottomColor.blue() < 40) & ((double) robot.bottomColor.green() < 40)) {
//                whileAutoPilot(4.71, 1.57, .75);
//            }

            stop();
        }
    }
}
