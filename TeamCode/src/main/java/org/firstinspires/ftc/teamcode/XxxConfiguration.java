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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PHRED_Bot;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="XX: Configuration Testing", group="Iterative Opmode")

public class XxxConfiguration extends OpMode
{
    PHRED_Bot robot = new PHRED_Bot();
    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

      /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.initializeRobot(hardwareMap);
    
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Show the elapsed game time 
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        
        // Shooter Testing
        if (gamepad1.left_bumper) {
            robot.shooterOn();
        } else {
            robot.shooterOff();
        }
        
        if (gamepad1.dpad_up) {
            //robot.liftMotor.setTargetPosition(25);
            robot.liftMotor.setPower(0.8);
        } else {
            robot.liftMotor.setPower(0.0);
        }
        
        if (gamepad1.dpad_down) {
            //robot.liftMotor.setTargetPosition(25);
            robot.liftMotor.setPower(-.3);
        } else {
            robot.liftMotor.setPower(0.0);
        }
        telemetry.addData("Lift Encoder", "position (%d)", robot.liftMotor.getCurrentPosition());
        
        // Servo Testing
        /*
        if(gamepad1.b) {
            robot.gripperServo.setPosition(robot.gripperServo.getPosition() + 0.1);
        }
        if(gamepad1.a) {
            robot.gripperServo.setPosition(robot.gripperServo.getPosition() - 0.1);
        }
        if(gamepad1.x) {
            robot.flipperServo.setPosition(robot.flipperServo.getPosition() + 0.1);
        }
        if(gamepad1.y) {
            robot.flipperServo.setPosition(robot.flipperServo.getPosition() - 0.1);
        }
        
        telemetry.addData("Servos Position", "gripper (%.2f), flipper (%.2f)", 
           robot.gripperServo.getPosition(),
           robot.flipperServo.getPosition());
        telemetry.addData("Servos Direction", "gripper %s, flipper %s",
           robot.gripperServo.getDirection(),
           robot.flipperServo.getDirection());
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        */
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
