/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.arcrobotics.ftclib.geometry.Transform2d;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//import com.spartronics4915.lib.T265Camera;


@TeleOp(name = "Tele", group = "Pushbot")

public class
PerseverenceTeleop extends LinearOpMode {
    HardwarePerseverence robot = new HardwarePerseverence();
    private BNO055IMU imu;
    private final ElapsedTime runtime = new ElapsedTime();
    private HardwareMap hardware = null;


    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CENTIMETERS = 10.16;     // For figuring circumference
    static final double COUNTS_PER_CENTIMETER = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CENTIMETERS * 3.1415);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 0.8;
    static final double DIST_PER_REV = (10 * Math.PI) / COUNTS_PER_MOTOR_REV;


    public void waitMilis(double timeOutMs) {

        runtime.reset();
        while (runtime.milliseconds() < timeOutMs) ;
    }

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Afz+Zrv/////AAABmSYDPqm+9Eeal4Qr3dPA93OKxmHLJOElO/flBfFW0x0sVdkLfIA4pm1EPqtd8fnTd0N0fKtVH0+hBxmglqS/Xn7S7Ahjpfd6sHeA4db+jCCf+lKvrt5LCuHp14NsBsjs4QB90u4hp/qGKQOyP4PVZ1KLC5gDu0VCP5vu4+M6505T4vAmojax18+tNGWInbVpUPqJuQPHregDhjsv7/6P/RLQMMK0zWW4zHCyiatZZX3NzGrrHvMh3uPE3mN/E0ucG/guL6Pp16PpLevoUXoz6IqqmWXcZUMo/IkRKdppCspWJbRfSngK5B2+MNzbM3tedVzFGaey/HApKBMxuqhO5uY+nWSud0GLd/SpJ9EmMRdl";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private final float phoneZRotate = 0;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */

    @Override
    public void runOpMode() {
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double drive;
        double turn;
        double max;
        double r;
        double robotAngle;
        double driveSpeed;
        int escape = 0;
        int choker = 0;
        double oldTime = 0.0;
        double oldTime1 = 0.0;
        boolean longEscape = false;
        double openEscape = 0;
        double closedEscape = .2;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuphoriaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuphoriaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuphoriaParameters.cameraName = robot.webcam;
        vuphoriaParameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(vuphoriaParameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField + 2, mmTargetHeight) /** Adjusted for our feild **/
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 8.5f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 13.5f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, vuphoriaParameters.cameraDirection);
        }

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
        // Send telemetry message to signify robot waiting;
        robot.escapeServo.setPosition(openEscape);
        telemetry.addData("Pushbot:", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            targetsUltimateGoal.activate();
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            // arm
            robot.arm.setPower(gamepad2.right_stick_y * .6);

            // choker
            if (gamepad2.x) {
                robot.choker.setPower(1.0);
            } else if (robot.chokerSwitch.getVoltage() < 3) {
                robot.choker.setPower(-1.0);
            } else robot.choker.setPower(0);

            telemetry.addData("voltage", robot.chokerSwitch.getVoltage());
            //escape
            switch (escape) {
                case 0:
                    escape++;
                    break;
                case 1:
                    if (gamepad2.a) {
                        oldTime = runtime.milliseconds();
                        robot.escapeServo.setPosition(openEscape);
                        robot.finalEscapeServo.setPosition(openEscape);
                        longEscape = false;
                        escape++;
                    } else if (gamepad2.b) {
                        oldTime = runtime.milliseconds();
                        robot.escapeServo.setPosition(openEscape);
                        robot.finalEscapeServo.setPosition(openEscape);
                        longEscape = true;
                        escape++;
                    }
                    break;
                case 2:
                    if (runtime.milliseconds() > oldTime + 400 && !longEscape) {
                        robot.finalEscapeServo.setPosition(closedEscape);
                        escape++;
                    } else if (runtime.milliseconds() > oldTime + 2500 && longEscape) {
                        robot.finalEscapeServo.setPosition(closedEscape);
                        escape++;
                    } else {
                        break;
                    }
                case 3:
                    if (robot.escapeSensor.getDistance(DistanceUnit.CM) > 5) {
                        robot.escapeServo.setPosition(openEscape);
                    } else if (robot.escapeSensor.getDistance(DistanceUnit.CM) < 5) {
                        telemetry.addData("Escapement Sensor", robot.escapeSensor.getDistance(DistanceUnit.CM));
                        robot.escapeServo.setPosition(closedEscape);
                        escape = 0;
                    } else {
                        break;
                    }
            }

            //Roller
            if (gamepad2.right_trigger > .5) {
                robot.rollers.setPower(1.0);
                robot.peewee.setPower(1.0);
            } else if (gamepad2.left_trigger > .5) {
                robot.rollers.setPower(-1.0);
                robot.peewee.setPower(-1.0);
            } else {
                robot.rollers.setPower(0.0);
                robot.peewee.setPower(0.0);
            }

            //aim
            //robot.aimbot.setPosition(.6);

            //looking glass/shooter off
            if (gamepad2.dpad_up) {
                robot.lookingGlass.setPower(1);
            } else if (gamepad2.dpad_down) {
                robot.lookingGlass.setPower(-1);
            } else if (gamepad2.dpad_right) {
                robot.lookingGlass.setPower(0);
                robot.flyWheel.setPower(0);
            }
            //shooter on
            if (gamepad2.dpad_left) {
                robot.flyWheel.setPower(1);
            }

            //power shots without camera
            if (gamepad2.y) {
                //shoot ring
                robot.flyWheel.setPower(.7);
                waitMilis(1500);

                autoPilot(0, 1.57, 6, 1, 2);

                robot.escapeServo.setPosition(openEscape);
                robot.finalEscapeServo.setPosition(openEscape);
                waitMilis(400);
                robot.escapeServo.setPosition(closedEscape);
                robot.finalEscapeServo.setPosition(closedEscape);
                robot.flyWheel.setPower(.75);

                autoPilot(0, 1.57, 17, 1, 10);
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
                waitMilis(500);

                robot.escapeServo.setPosition(openEscape);
                robot.finalEscapeServo.setPosition(openEscape);
                waitMilis(400);
                robot.escapeServo.setPosition(closedEscape);
                robot.finalEscapeServo.setPosition(closedEscape);

                autoPilot(0, 1.57, 17, 1, 10);
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
                waitMilis(500);

                robot.escapeServo.setPosition(openEscape);
                robot.finalEscapeServo.setPosition(openEscape);
                waitMilis(400);
                robot.escapeServo.setPosition(closedEscape);
                robot.finalEscapeServo.setPosition(closedEscape);

                robot.escapeServo.setPosition(openEscape);
                robot.flyWheel.setPower(1);
            }

            // slow mode
            if (gamepad1.right_trigger > 0.5) {
                driveSpeed = 0.5;
            } else {
                driveSpeed = 1;
            }

            // drive
            double rightX = gamepad1.right_stick_x;
            r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            robot.leftDrive.setPower(v1 * driveSpeed);
            robot.rightDrive.setPower(v2 * driveSpeed);
            robot.leftBackDrive.setPower(v3 * driveSpeed);
            robot.rightBackDrive.setPower(v4 * driveSpeed);
            // Send telemetry message to signify robot running;
            double currentHeading = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double headingRadians = -((-currentHeading / 180) * 3.1416) + (1 / 2 * 3.1416);

            telemetry.addData("Heading", headingRadians);
            telemetry.addData("RightColor", robot.rightBottomColor.alpha());
            telemetry.addData("LeftColor", robot.leftBottomColor.alpha());
            // Provide feedback as to where the robot is located (if we know).
            autoAim();
            telemetry.update();
        }
        targetsUltimateGoal.deactivate();
    }

    public void autoAim() {
//        telemetry.addData("x", x);
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            double x1 = 3;
            double y1 = -14;

            //double x2 = -72 + (robot.frontDistance.getDistance(DistanceUnit.INCH) + 8.1);
            double x2 = translation.get(0);
            double y2 = translation.get(1);

            double dx;
            double dy;
            double translationDistance;
            double translationAngle;

            if (false) {
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                runtime.reset();
//                while (robot.frontDistance.getDistance(DistanceUnit.INCH) > 322 && runtime.seconds() < 5) {
//                    robot.leftDrive.setPower(1.0);
//                    robot.rightDrive.setPower(1.0);
//                    robot.leftBackDrive.setPower(1.0);
//                    robot.rightBackDrive.setPower(1.0);
//                }
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
                dx = x1 - x2;
                dy = y1 - y2;
                translationDistance = (dx * dx) + (dy * dy);
                translationDistance = Math.sqrt(translationDistance);
                telemetry.addData("x1", x1);
                telemetry.addData("x2", x2);
                telemetry.addData("y1", y1);
                telemetry.addData("y2", y2);
                telemetry.addData("dx", dx);
                telemetry.addData("dy", dy);
                translationAngle = Math.atan(dy / dx);
                translationAngle = (translationAngle * (Math.PI / 180));
                translationAngle = (4.71 - translationAngle);
                //  translationAngle = ((180 - (translationAngle)) * (Math.PI / 180));
                telemetry.addData("Distance", translationDistance);
                telemetry.addData("Angle", translationAngle * (180 / Math.PI));
                telemetry.addData("Angle Before", Math.atan(dy / dx));
                telemetry.addData("Angle Raidians", translationAngle);
                //autoPilot(translationAngle, 1.57, translationDistance * 2.54, .25, 10);
            }


        } else {
            telemetry.addData("Visible Target", "none");
        }
    }

    public void imuReset() {
        while (robot.rightBottomColor.alpha() < 1000 && robot.leftBottomColor.alpha() < 1000) {
            robot.leftDrive.setPower(0.25);
            robot.rightDrive.setPower(0.25);
            robot.leftBackDrive.setPower(0.25);
            robot.rightBackDrive.setPower(0.25);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        while (robot.rightBottomColor.alpha() < 1000) {
            robot.rightDrive.setPower(0.25);
            robot.rightBackDrive.setPower(0.25);
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        while (robot.leftBottomColor.alpha() < 1000) {
            robot.rightDrive.setPower(0.25);
            robot.rightBackDrive.setPower(0.25);
        }
        imu.startAccelerationIntegration(null, null, 10);
        return;
    }

    public double subtractAngle(double angleA,
                                double angleB) {
        double result;
        result = angleA - angleB;
        if (result > 180) {
            result = result - 360;
        }
        return result;
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

}


