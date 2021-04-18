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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.PHRED_Bot;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="xx Test: Linear OpMode", group="Autonomous")

public class XxxDriveTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private PHRED_Bot robot = new PHRED_Bot();
    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.initializeRobot(hardwareMap);
        
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

             while (runtime.milliseconds() < 1000) {
                             telemetry.addData("deviceName",robot.frontRangeSensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", robot.frontRangeSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", robot.frontRangeSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", robot.frontRangeSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", robot.frontRangeSensor.getDistance(DistanceUnit.INCH)));

            telemetry.addData("angle", String.format("%.1f deg", robot.angle()));
telemetry.update();
                robot.driveForward(.8);
            }
            while (runtime.milliseconds() < 2000) {
                robot.driveLeft(.8);
                            telemetry.addData("deviceName",robot.frontRangeSensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", robot.frontRangeSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", robot.frontRangeSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", robot.frontRangeSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", robot.frontRangeSensor.getDistance(DistanceUnit.INCH)));

            telemetry.addData("angle", String.format("%.1f deg", robot.angle()));
telemetry.update();

            }
            while (runtime.milliseconds() < 3000) {
                robot.driveRight(.8);
                            telemetry.addData("deviceName",robot.frontRangeSensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", robot.frontRangeSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", robot.frontRangeSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", robot.frontRangeSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", robot.frontRangeSensor.getDistance(DistanceUnit.INCH)));

            telemetry.addData("angle", String.format("%.1f deg", robot.angle()));
telemetry.update();

            }
            while (runtime.milliseconds() < 4000) {
                robot.driveBackwards(.8);
                            telemetry.addData("deviceName",robot.frontRangeSensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", robot.frontRangeSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", robot.frontRangeSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", robot.frontRangeSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", robot.frontRangeSensor.getDistance(DistanceUnit.INCH)));

            telemetry.addData("angle", String.format("%.1f deg", robot.angle()));
telemetry.update();

            }
            while (runtime.milliseconds() < 5000) {
                robot.turnRight(.8);
                            telemetry.addData("deviceName",robot.frontRangeSensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", robot.frontRangeSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", robot.frontRangeSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", robot.frontRangeSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", robot.frontRangeSensor.getDistance(DistanceUnit.INCH)));

            telemetry.addData("angle", String.format("%.1f deg", robot.angle()));
telemetry.update();

            }
            while (runtime.milliseconds() < 6000) {
                robot.turnLeft(.8);
                            telemetry.addData("deviceName",robot.frontRangeSensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", robot.frontRangeSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", robot.frontRangeSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", robot.frontRangeSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", robot.frontRangeSensor.getDistance(DistanceUnit.INCH)));

            telemetry.addData("angle", String.format("%.1f deg", robot.angle()));
telemetry.update();

            }
            robot.stopRobot();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
}
