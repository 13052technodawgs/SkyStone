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

package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;

@TeleOp(name="Holonomic Drive", group="Pushbot")
public class DEPRECATEDHolonomicDriveTeleop extends OpMode{

    final int rotMult = -1;

    boolean grabberLock = false;
    boolean lastX = false;

    /* Declare OpMode members. */
    DEPRECATEDHardwareTechnoDawgs robot       = new DEPRECATEDHardwareTechnoDawgs(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        if(!robot.homeSensor.getState()){
            telemetry.addData("Home", "True");
        }else{
            telemetry.addData("Home", "False");
//            throw new Error("Robot is not home! Reset to start position");
            throw new ExceptionWithContext("Robot is not home! Reset to start position");
        }
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if(!robot.homeSensor.getState()){
            telemetry.addData("Home", "True");
        }else{
            telemetry.addData("Home", "False");
            throw new ExceptionWithContext("Robot is not home! Reset to start position");
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // INPUT
        double x;
        double y;
        double rot;

        double fl;
        double fr;
        double bl;
        double br;

        double armPos;
        double grabberPos;


        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        rot = gamepad1.right_stick_x;
        boolean buttonX = gamepad2.x;

        // Debounce x button and activate grabber lock
        if(buttonX && !lastX){
            if(gamepad2.right_trigger>0.5 && !grabberLock){
                grabberLock = true;
            } else if (grabberLock) {
                grabberLock = false;
            }
        }

        if (gamepad2.dpad_up) robot.hookServo.setPosition(1);
        if (gamepad2.dpad_down) robot.hookServo.setPosition(0.3);

        // Square input to smooth input values
        x*=Math.abs(x);
        y*=Math.abs(y);
        rot*=Math.abs(rot);

        armPos = gamepad2.left_stick_y;
        grabberPos = gamepad2.right_trigger>0.5 || grabberLock? 1: 0.5;

        //OUTPUT
        fl = x* DEPRECATEDRobotDirection.RIGHT.FL() + y* DEPRECATEDRobotDirection.FORWARD.FL() + rot*rotMult;
        fr = x* DEPRECATEDRobotDirection.RIGHT.FR() + y* DEPRECATEDRobotDirection.FORWARD.FR() + rot*rotMult;
        bl = x* DEPRECATEDRobotDirection.RIGHT.BL() + y* DEPRECATEDRobotDirection.FORWARD.BL() + rot*rotMult;
        br = x* DEPRECATEDRobotDirection.RIGHT.BR() + y* DEPRECATEDRobotDirection.FORWARD.BR() + rot*rotMult;

        //TODO: calculate arm direction and speed, run to that position
        //TODO: Set servo position
        //1120 ticks = 180 motor degrees
        //Gear reduction 1:6
        //6720 ticks = 180 arm degrees

        //TODO: scale from -1 to 0 to 1 -> 0 to 1.5 to 2

        final double restPos = 5.0/6.0;

        if(armPos>0){
            armPos*=2*(1-restPos);
        }else{
            armPos*=2*restPos;
        }

        armPos += restPos*2; //scale from -1 to 1 -> 0 to 2

        final double circleFraction = 0.77;

        armPos *= 1120*6/2 * circleFraction;

        double armPower = armPos>robot.armMotor.getCurrentPosition()? -0.6: 0.6;

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

        // Send telemetry message to signify robot running;
        telemetry.addData("x",  "%.2f", x);
        telemetry.addData("y", "%.2f", y);
        telemetry.addData("rot", "%.2f", rot);
        telemetry.addData("armGoal", "%.2f", armPos);
        try {
            telemetry.addData("armPosition", "%d", robot.armMotor.getCurrentPosition());
            telemetry.addData("lastX", "%b", lastX);
            telemetry.addData("buttonX", "%b", buttonX);
            telemetry.addData("lock", "%b", grabberLock);

        } catch (Exception e) {
            //doNada
        }

        lastX = buttonX;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }
}
