package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwarePerseverence;

@TeleOp(name = "Tele", group = "TeleOp")


public class PerseverenceMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwarePerseverence robot = new HardwarePerseverence();
        final ElapsedTime runtime = new ElapsedTime();
        robot.init(hardwareMap);
        double r;
        double robotAngle;
        double driveSpeed = 1;
//        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Testbot:","Waiting for start");
        telemetry.update();
        waitForStart();
       while (opModeIsActive())  {
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
        }
    }
}
