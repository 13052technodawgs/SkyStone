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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="ADVANCED BLUE", group="Technodawgs")
public class AdvancedBlueAuto extends LinearOpMode {
    HardwareTechnoDawgs robot = new HardwareTechnoDawgs();
    Orientation lastAngles = new Orientation();
    private double globalAngle, power = .70, correction;

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         */
        robot.init(hardwareMap);

        // Wait for the game to start
        while (!isStarted()) {

        }

        {
            //AUTONOMOUS MOTION SEQUENCE
            //PUT YOUR AUTO CODE HERE

            moveStraight(180.0, RobotDirection.LEFT);
            moveStraight(1.75*360.0, RobotDirection.FORWARD);
//            moveStraight(720.0, DEPRECATEDRobotDirection.BACKWARD);
//            moveStraight(720.0, DEPRECATEDRobotDirection.RIGHT);
        }


        robot.stopMotors( );

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }
    // METHODS
    /**
     * go in a straight line using IMU to correct for rotation
     * 2240 pulses per rotation of output shaft
     *
     * @param wheelRotationInDegrees wheel rotation angle in degrees
     * @param direction the direction of movement
     */
    private void moveStraight(double wheelRotationInDegrees, RobotDirection direction){

        resetAngle();

        int pulses = direction.FL() * (int)((wheelRotationInDegrees/360.0)*2240);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setTargetPosition(pulses);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() && robot.frontLeft.isBusy()) {
            correction = checkDirection();

            robot.frontLeft.setPower(  direction.FL() * power + correction);
            robot.frontRight.setPower( direction.FR() * power + correction);
            robot.backLeft.setPower(   direction.BL() * power + correction);
            robot.backRight.setPower(  direction.BR() * power + correction);
        }
        robot.stopMotors( );

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
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
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
}

