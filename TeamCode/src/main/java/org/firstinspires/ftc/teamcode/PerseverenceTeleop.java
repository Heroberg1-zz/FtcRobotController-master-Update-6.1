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


@TeleOp(name = "Simp Tele", group = "Pushbot")

public class
PerseverenceTeleop extends LinearOpMode {
    HardwarePerseverence robot = new HardwarePerseverence();
    private BNO055IMU imu;
    private final ElapsedTime runtime = new ElapsedTime();
    private HardwareMap hardware = null;


    static final double COUNTS_PER_MOTOR_REV = 383.6;
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
    double openEscape = 0;
    double closedEscape = .3;
    double finalOpenEscape = .05;
    double finalClosedEscape = .255;


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
//        double openEscape = 0;
//        double closedEscape = .3;
//        double finalOpenEscape = 0;
//        double finalClosedEscape = .25;
        boolean flywheelOn = false;
        boolean chokerClosed = false;
        boolean flyWheelSlow = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

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
        robot.flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //run using the encoders
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        robot.chopper.setPosition(0.9);
        robot.escapeServo.setPosition(openEscape);
        robot.finalEscapeServo.setPosition(finalOpenEscape);
        telemetry.addData("Pushbot:", "Hello Driver");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // arm
            robot.arm.setPower(gamepad2.left_stick_y * .5);

            // choker
            if (gamepad2.x) {
                robot.choker.setPosition(.4125);
            } else {
                robot.choker.setPosition(0);
            }
            //escape
            switch (escape) {
                case 0:
                    escape++;
                    break;
                case 1:
                    if (gamepad2.a) {
                        oldTime = runtime.milliseconds();
                        robot.escapeServo.setPosition(openEscape);
                        robot.finalEscapeServo.setPosition(finalOpenEscape);
                        longEscape = false;
                        escape++;
                    } else if (gamepad2.b) {
                        oldTime = runtime.milliseconds();
                        robot.escapeServo.setPosition(openEscape);
                        robot.finalEscapeServo.setPosition(finalOpenEscape);
                        longEscape = true;
                        escape++;
                    }
                    break;
                case 2:
                    if (runtime.milliseconds() > oldTime + 400 && !longEscape) {
                        robot.finalEscapeServo.setPosition(finalClosedEscape);
                        escape++;
                    } else if (runtime.milliseconds() > oldTime + 2500 && longEscape) {
                        robot.finalEscapeServo.setPosition(finalClosedEscape);
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
                robot.flyWheel.setPower(1);
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
            //chopper
            if (gamepad1.left_trigger > 0.5) {
                robot.chopper.setPosition(.03);
            } else {
                robot.chopper.setPosition(.9);
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
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Flywheel Power", robot.flyWheel.getPower());
            telemetry.update();
        }

    }
}


