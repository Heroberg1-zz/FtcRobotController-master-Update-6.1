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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePerseverence {
    /* Public OpMode members. */
    /**
     * Motors
     */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor peewee = null;
    public DcMotor rollers = null;
    public DcMotor lookingGlass = null;
    public DcMotor flyWheel = null;
    /**
     * Servos
     */
    public CRServo arm = null;
    public Servo choker = null;
    public Servo aimbot = null;
    public Servo escapeServo = null;
    public Servo finalEscapeServo = null;
    public Servo chopper = null;
    /** Analog */
    /**
     * Digital
     */
    public AnalogInput chokerSwitch = null;
    public AnalogInput sideSwitch = null;
    //   public AnalogInput leftMarker = null;
    //  public AnalogInput rightMarker = null;
    /**
     * I2C
     */
    public DistanceSensor escapeSensor = null;
    public ColorSensor rightBottomColor = null;
    public ColorSensor leftBottomColor = null;
    public DistanceSensor frontDistance = null;
    /**
     * USB
     */
    public CameraName webcam = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private final ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwarePerseverence() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /** Motors */
        leftDrive = hwMap.get(DcMotor.class, "ld");
        rightDrive = hwMap.get(DcMotor.class, "rd");
        leftBackDrive = hwMap.get(DcMotor.class, "lbd");
        rightBackDrive = hwMap.get(DcMotor.class, "rbd");
        peewee = hwMap.get(DcMotor.class, "peewee");
        rollers = hwMap.get(DcMotor.class, "roller");
        lookingGlass = hwMap.get(DcMotor.class, "lookingGlass");
        flyWheel = hwMap.get(DcMotor.class, "flyWheel");
        /** Servos */
        arm = hwMap.get(CRServo.class, "arm");
        choker = hwMap.get(Servo.class, "choker");
        aimbot = hwMap.get(Servo.class, "aimbot");
        escapeServo = hwMap.get(Servo.class, "escapeServo");
        finalEscapeServo = hwMap.get(Servo.class, "finalEscapeServo");
        chopper = hwMap.get(Servo.class, "chopper");
        /** Analog */
        /** Digital */
        chokerSwitch = hwMap.get(AnalogInput.class, "chokerSwitch");
        sideSwitch = hwMap.get(AnalogInput.class, "wallSwitch");
        //  leftMarker = hwMap.get(AnalogInput.class, "leftMarker");
        //   rightMarker = hwMap.get(AnalogInput.class, "rightMarker");
        /** I2C */
        rightBottomColor = hwMap.get(ColorSensor.class, "bottomColor");
        leftBottomColor = hwMap.get(ColorSensor.class, "leftBottomColor");
        escapeSensor = hwMap.get(DistanceSensor.class, "escapeSensor");
        frontDistance = hwMap.get(DistanceSensor.class, "frontDistance");
        /** USB */
        webcam = hwMap.get(WebcamName.class, "webcam");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rollers.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        peewee.setPower(0);
        rollers.setPower(0);
        lookingGlass.setPower(0);
        flyWheel.setPower(0);
        // Set all motors to run without encoders.
        peewee.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lookingGlass.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //choker.scaleRange(.3,.5);


        // Define and initialize ALL installed servos.

    }
}

