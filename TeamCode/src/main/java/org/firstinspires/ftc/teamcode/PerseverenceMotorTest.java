package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwarePerseverence;

@TeleOp(name = "Motor Test", group = "Pushbot")
public class PerseverenceMotorTest extends LinearOpMode {
    HardwarePerseverence robot = new HardwarePerseverence();
    private final ElapsedTime runtime = new ElapsedTime();

    public void waitMilis(double timeOutMs) {

        runtime.reset();
        while (runtime.milliseconds() < timeOutMs) ;
    }

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        telemetry.addData("Ready?", "Press A");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        telemetry.addData("Rollers", "Press A for positive 25%");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        robot.rollers.setPower(.25);
        waitMilis(5000);
        robot.rollers.setPower(0);
        telemetry.addData("Tendrails", "Press A for positive 25%");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        robot.tendrails.setPower(.25);
        waitMilis(5000);
        robot.tendrails.setPower(0);
        telemetry.addData("LookingGlass", "Press A for positive 25%");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        robot.lookingGlass.setPower(.25);
        waitMilis(5000);
        robot.lookingGlass.setPower(0);
        telemetry.addData("Flywheel", "Press A for positive 25%");
        telemetry.update();
        while (!gamepad1.a) {
            waitMilis(50);
        }
        robot.flyWheel.setPower(.25);
        waitMilis(5000);
        robot.flyWheel.setPower(0);
    }
}
