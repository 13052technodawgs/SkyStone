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
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Holonomic Drive v2", group="Technodawgs")
public class HolonomicDriveTeleop extends OpMode{
    // CONSTANTS
    final double restPos = 5.0/6.0;
    final double armArc = 0.77;
    final File soundFile = new File("/sdcard/Audio/WilhelmScream.wav");

    // INPUTS
    double movementX;
    double movementY;
    double movementRot;

    int buttonPresses;

    double moveAngle;

    Deadline chargeClock = new Deadline(1000, TimeUnit.MILLISECONDS);
    Deadline gunClock = new Deadline(1000, TimeUnit.MILLISECONDS);

    // OUTPUTS
    double fl;
    double fr;
    double bl;
    double br;

    double armPos;
    double armPower;
    double grabberPos;
    boolean grabberLock = false;

    double dishAngle;

    RevBlinkinLedDriver.BlinkinPattern ledPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    // FIELDS
    GamepadExtender controller1;
    GamepadExtender controller2;

    HardwareTechnoDawgs robot       = new HardwareTechnoDawgs(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        controller1 = new GamepadExtender(gamepad1);
        controller2 = new GamepadExtender(gamepad2);

        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.armMotor.setPower(0);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        controller1.update();

        if(!robot.homeSensor.getState()){
            telemetry.addData("Home", "True");
            robot.ledServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }else{
            telemetry.addData("Home", "False");
            robot.ledServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        }

        if(controller1.xPressed()){
            ledPattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
        }
        if(controller1.bPressed()){
            ledPattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        }

        switch (ledPattern){
            case HEARTBEAT_BLUE:
                telemetry.addData("Alliance", "BLUE");
                break;
            case HEARTBEAT_RED:
                telemetry.addData("Alliance", "RED");
                break;
            default:
                telemetry.addData("Alliance", "NONE");
        }

        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(0);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        buttonPresses = 0;
        // Check to make sure robot is in starting position
        if(!robot.homeSensor.getState()){
            telemetry.addData("Home", "True");
        }else{
            telemetry.addData("Home", "False");
            throw new ExceptionWithContext("Robot is not home! Reset to start position");
        }
        telemetry.update();

        robot.ledServo.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        input();
        process();
        output();

        //===============================================================
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stopMotors();
    }

    /**
     * Collect input from all sensors and controllers to prepare for
     * robot processing.
     *
     * Having this separate ensures we keep input consistent for each
     * iteration of the loop, rather than changing input affecting
     * robot behavior adversely in the middle of its run.
     */
    private void input() {
        controller1.update();
        controller2.update();

        movementX = controller1.getLeftStickX();
        movementY = -controller1.getLeftStickY();
        movementRot = controller1.getRightStickX();

        // Square input to smooth input values
        movementX   *= Math.abs(movementX);
        movementY   *= Math.abs(movementY);
        movementRot *= Math.abs(movementRot);

        armPos = controller2.getLeftStickY();

        moveAngle = Math.atan(movementX/movementY);
        if(Double.isNaN(moveAngle)) moveAngle=Math.PI;
        moveAngle = Math.toDegrees(moveAngle);


    }

    /**
     * Process the input, performing whatever math and logic is necessary
     * to find the values to output.
     */
    private void process() {
        grabberLock = controller2.xPressed() && (controller2.getRightTrigger()>0.5);
        grabberPos = (controller2.getRightTrigger()>0.5 || grabberLock)? 1: 0.5;

        if (controller2.dUpPressed()) robot.hookServo.setPosition(1);
        if (controller2.dDownPressed()) robot.hookServo.setPosition(0.3);

        fl = movementX* RobotDirection.RIGHT.FL() + movementY * RobotDirection.FORWARD.FL() - movementRot;
        fr = movementX* RobotDirection.RIGHT.FR() + movementY * RobotDirection.FORWARD.FR() - movementRot;
        bl = movementX* RobotDirection.RIGHT.BL() + movementY * RobotDirection.FORWARD.BL() - movementRot;
        br = movementX* RobotDirection.RIGHT.BR() + movementY * RobotDirection.FORWARD.BR() - movementRot;

        if(controller2.aPressed()){
            SoundPlayer.getInstance().startPlaying(robot.hwMap.appContext, soundFile);
        }

        //1120 ticks = 180 motor degrees
        //Gear reduction 1:6
        //6720 ticks = 180 arm degrees

        // Scale armPos based on restPos, so half the input is a greater fraction of the output
        if(armPos>0){
            armPos*=2*(1-restPos);
        }else{
            armPos*=2*restPos;
        }

        armPos += restPos*2; //scale from -1 to 1 -> 0 to 2
        armPos /= 2;         //scale from  0 to 2 -> 0 to 1
        armPos *= 1120*6 * armArc;

        // Move the arm in the correct direction
        armPower = (armPos>robot.armMotor.getCurrentPosition())? -0.6: 0.6;

        dishAngle = (moveAngle+60)/(300-60);

        if(dishAngle<0)dishAngle=0;
        if(dishAngle>1)dishAngle=1;

    }

    /**
     * Give values to the robot hardware so it responds to input accordingly
     */
    private void output() {
        robot.armMotor.setTargetPosition((int)armPos);
        robot.armMotor.setPower(armPower);

        robot.frontLeft.setPower(fl);
        robot.frontRight.setPower(fr);
        robot.backLeft.setPower(bl);
        robot.backRight.setPower(br);

        // get the robot's front servo and set its position to grabberPos
        // Do the same with the backServo
        robot.frontServo.setPosition(grabberPos);
        robot.backServo.setPosition(1-grabberPos);

        robot.dishServo.setPosition(dishAngle);

        robot.ledServo.setPattern(ledPattern);

        // Send telemetry message to signify robot running;
        telemetry.addData("movementX",  "%.2f", movementX);
        telemetry.addData("movementY", "%.2f", movementY);
        telemetry.addData("rotation", "%.2f", movementRot);
        telemetry.addData("armGoal", "%.2f", armPos);
        telemetry.addData("armPosition", "%d", robot.armMotor.getCurrentPosition());
        telemetry.addData("lock", "%b", grabberLock);
    }

}
