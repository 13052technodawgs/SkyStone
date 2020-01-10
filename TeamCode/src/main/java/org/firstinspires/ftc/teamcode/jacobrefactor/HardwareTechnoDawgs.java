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

package org.firstinspires.ftc.teamcode.jacobrefactor;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.util.concurrent.TimeUnit;

/**
 * This class is a wrapper for the management of all robot hardware
 *
 * DRIVE MOTORS
 * Motor:   frontLeft
 * Motor:   frontRight
 * Motor:   backLeft
 * Motor:   backRight
 *
 * GAME PIECE PARTS
 * Motor:   armMotor
 * Servo:   frontServo
 * Servo:   backServo
 * Servo:   hookServo
 *
 * Sensor:  homeSensor
 * IMU:     imu
 */
public class HardwareTechnoDawgs {

    /* Public OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    public DcMotor armMotor = null;

    public Servo frontServo = null;
    public Servo backServo = null;
    public Servo hookServo = null;
    public RevBlinkinLedDriver ledServo = null;
    public Servo dishServo = null;

    public DigitalChannel homeSensor = null;
    public BNO055IMU imu;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    HardwareMap hwMap = null;
    private LinearOpMode thisOpMode = null;
    private ElapsedTime period = new ElapsedTime();

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .70, correction;


    Deadline klaxonClock = new Deadline(6000, TimeUnit.MILLISECONDS);
    final File klaxonFile = new File("/sdcard/Audio/klaxon.wav");


    /* Constructor */
    public HardwareTechnoDawgs() {

    }

    public void init(HardwareMap hwMap, LinearOpMode thisOpMode){
        init(hwMap);
        this.thisOpMode = thisOpMode;
    }

    /**
     * Initialize all the hardware interfaces using the hardware map.
     * This should be done prior to attempting to control any part of the robot.
     *
     * @param hwMap the hardware map for the robot
     */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        this.hwMap = hwMap;

        // Define Motors, Servos, and Sensors from Hardware Mapping
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        armMotor = hwMap.get(DcMotor.class, "armMotor");

        frontServo = hwMap.get(Servo.class,"frontServo");
        backServo = hwMap.get(Servo.class,"backServo");
        hookServo = hwMap.get(Servo.class, "hookServo");
        ledServo = hwMap.get(RevBlinkinLedDriver.class, "ledServo");
        dishServo = hwMap.get(Servo.class,"dishServo");

        homeSensor = hwMap.get(DigitalChannel.class, "homeSensor");
        imu = hwMap.get(BNO055IMU.class, "imu");

        // Reverse the armMotor for sensible control
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all drive motors to run using their encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the arm to run to position, like a servo
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        homeSensor.setMode(DigitalChannel.Mode.INPUT);

        // Intiialize the IMU and wait for its Gyro to be calibrated
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());
    }

    /**
     * To be used in autonomous, emergency stops, or any time it is
     * important to make sure no motors are running
     */
    public void stopMotors(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        armMotor.setPower(0);
    }


    @Deprecated
    public void moveStraight(double wheelRotationInDegrees, RobotDirection direction){
        resetAngle();

        int pulses = direction.FL() * (int)((wheelRotationInDegrees/360.0)*2240);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(pulses);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(thisOpMode.opModeIsActive() && frontLeft.isBusy()) {
            correction = checkDirection();

            frontLeft.setPower(  direction.FL() * power + correction);
            frontRight.setPower( direction.FR() * power + correction);
            backLeft.setPower(   direction.BL() * power + correction);
            backRight.setPower(  direction.BR() * power + correction);
        }
        stopMotors( );

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(double wheelRotationInDegrees, RobotDirection direction){

        int pulses = direction.FL() * (int)((wheelRotationInDegrees/360.0)*2240);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(pulses);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(thisOpMode.opModeIsActive() && frontLeft.isBusy()) {
            frontLeft.setPower(  direction.FL() * power);
            frontRight.setPower( direction.FR() * power);
            backLeft.setPower(   direction.BL() * power);
            backRight.setPower(  direction.BR() * power);
        }
        stopMotors( );

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a
     * power correction value.
     * @return Power adjustment, + is adjust counterclockwise - is adjust clockwise.
     */
    private double checkDirection()
    {
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void armCheck(){
        if(!homeSensor.getState()){
            ledServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            SoundPlayer.getInstance().stopPlayingAll();
            klaxonClock.expire();
        }else{
            ledServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            if(klaxonClock.hasExpired()){
                klaxonClock.reset();
                SoundPlayer.getInstance().startPlaying(hwMap.appContext, klaxonFile);
            }
        }
    }

    public void teamSet(GamepadExtender controller1, RevBlinkinLedDriver.BlinkinPattern ledPattern){
        if(controller1.xPressed()){
            ledPattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE;
        }
        if(controller1.bPressed()){
            ledPattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_RED;
        }

        switch (ledPattern){
            case SHOT_BLUE:
                thisOpMode.telemetry.addData("Alliance", "BLUE");
                break;
            case SHOT_RED:
                thisOpMode.telemetry.addData("Alliance", "RED");
                break;
            default:
                thisOpMode.telemetry.addData("Alliance", "NONE");
        }
    }
}

