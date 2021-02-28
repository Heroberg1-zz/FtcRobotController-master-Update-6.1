package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwarePerseverence;

@TeleOp(name = "Motor Test", group = "TeleOp")
public class PerseverenceMotorTest extends LinearOpMode {
    HardwarePerseverence robot = new HardwarePerseverence();
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        waitForStart();
       while (opModeIsActive())  {
           robot.rightBackDrive.setPower(1.0);
//           telemetry.addData("Centimeters",robot.escapeSensor.getDistance(DistanceUnit.CM));
//           telemetry.addData("Heading First Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//           telemetry.addData("Heading Second Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
//           telemetry.addData("Heading Third Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
//           telemetry.update();
//           while (robot.rightBottomColor.alpha() < 1000 || robot.leftBottomColor.alpha() < 1000 && runtime.seconds() < 6) {
//               whileAutoPilot(1.57, 1.57, .4);
//           }
        }
    }
}
