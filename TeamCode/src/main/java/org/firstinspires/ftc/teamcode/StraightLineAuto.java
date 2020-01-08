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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Pushbot: Auto Drive By Gyro", group="Pushbot")
//@Disabled
public class StraightLineAuto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTechnoDawgs robot = new HardwareTechnoDawgs();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device
    // VARIABLES HERE
    //VARIABLES
    Orientation lastAngles = new Orientation();
    private double globalAngle, power = .70, correction;


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {

        }

        moveStraight(7200.0, RobotDirection.FORWARD);
        moveStraight(7200.0, RobotDirection.LEFT);
        moveStraight(7200.0, RobotDirection.BACKWARD);
        moveStraight(7200.0, RobotDirection.RIGHT);

       robot.stop( );

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    // METHODS HERE
//METHODS
    /**
     * go forward with gyro correction
     * 2240 pulses per rotation of output shaft
     *
     * @param angle wheel rotation angle in degrees
     */
    private void moveStraight(double angle, RobotDirection direction){
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int pulses = direction.FL() * (int)((angle/360.0)*2240);

        robot.frontLeft.setTargetPosition(pulses);

        while(opModeIsActive() && robot.frontLeft.isBusy()) {
            correction = checkDirection();

            robot.frontLeft.setPower(  direction.FL() * power + correction);
            robot.frontRight.setPower( direction.FR() * power + correction);
            robot.backLeft.setPower(   direction.BL() * power + correction);
            robot.backRight.setPower(  direction.BR() * power + correction);
        }
        robot.stop( );

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the
        // axis we want to use for heading angle.
        // We have to process the angle because the imu works
        // in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to
        // -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total
        // cumulative angle of rotation.

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
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction
        // is to direction changes.
        // You will have to experiment with your robot to get small
        // smooth direction changes
        // to stay on a straight line.
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
