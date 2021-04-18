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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PHRED_Bot;


/**
 * This is the Main Teleop program for the PHRED FTC Robot
 *
 * Controls:
 *   Gamepad 1:
 *     Left Joystick:
 *       Forward/Back   - Drive the Robot Forward/Reverse
 *       Left/Right     - "Slide" the Robot Left/Right
 *     Right Joystick
 *       Left/Right     - Turn the robot Counter-Clockwise/Clockwise
 *     Left Trigger     - Drive the lift arm up
 *     Left Bumper      - Reset the lift arm encoder
 *
 *   Gamepad 2:
 * 
 * Actions:
 * Close/Open grabber
 * forward/back flipper
 * turn on shooters
 * tilter up/back
 * With the Tilt Arm at top - Reset encoder
 * 
 * Drive to shooting line
 */

@TeleOp(name = "TeleOp: Main TeleOp", group = "Iterative Opmode")

public class MainTeleOp extends OpMode {

    // Declare Constants

    // Declare OpMode members

    PHRED_Bot robot = new PHRED_Bot();

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime shootCycle = new ElapsedTime();

    private double frontRightPower = 0.0;
    private double backRightPower = 0.0;
    private double frontLeftPower = 0.0;
    private double backLeftPower = 0.0;

    // Debounce variables
    private boolean dpad2UpPressed = false;
    private boolean dpad2DownPressed = false;

    private boolean shooting = false;
    private boolean rightBumper2Press = false;
    
    
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
        int liftEncoder;

        // Mechanum Mode use Left Stick for motion and Right Stick to rotate
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // - This uses basic math to combine motions and is easier to drive straight.
        frontLeftPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        frontRightPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        backLeftPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        backRightPower = Range.clip(drive + turn - strafe, -1.0, 1.0);

        // Now Drive the Robot
        robot.driveRobot(frontRightPower, frontLeftPower, backRightPower, backLeftPower);
        
        // --- Servo Controls ---
        // Grabber Servo Open/Close
        if(gamepad2.x) {
            robot.gripperServo.setPosition(0);
        }

        if (gamepad2.y) {
            robot.gripperServo.setPosition(1);
        }

        
        // Lifter Arm Routines
        //  DON'T LOCK UP OTHER ACTIONS
        if (gamepad2.dpad_up 
              && !dpad2UpPressed ) {
            // first press of dpad_up so run to position(up)
            robot.liftMotor.setTargetPosition(robot.LIFT_TOP);
            robot.liftMotor.setPower(robot.LIFT_SPEED_MAX);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            
            dpad2UpPressed = true;
        } 

        if (gamepad2.dpad_down 
              && !dpad2DownPressed ) {
            // first press of dpad_down so run to position(down)
            robot.liftMotor.setTargetPosition(robot.LIFT_BOTTOM);
            robot.liftMotor.setPower(-.8);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
            
            
            dpad2DownPressed = true;
        } 
        if (!gamepad2.dpad_up && robot.liftMotor.getCurrentPosition() < 10 && robot.liftMotor.getCurrentPosition() != 0){
            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        
        if ((dpad2UpPressed || dpad2DownPressed) // if started lift arm
             && !robot.liftMotor.isBusy()) {     // and now done
            robot.liftMotor.setPower(0);
            dpad2UpPressed = false;
            dpad2DownPressed = false;
        }
        
        // Shooting
        if (gamepad2.right_bumper && !rightBumper2Press) {
            // Turn Motors on
            robot.shooterOn();

            // set a timer
            shootCycle.reset();
            shooting = true;
        }
        
            if (gamepad2.right_bumper) {
                if ( shootCycle.milliseconds() > 500) {
                   robot.flipperServo.setPosition(1);
                }
            } else {
                robot.flipperServo.setPosition(0);
            }
        
        
        // Turn off shooting
        if (shooting && shootCycle.milliseconds() > robot.SHOOTING_MILLISECONDS) {
            shooting = false;
            robot.shooterOff();
        }
        
        
        /* ------   U t i l i t y   R o u t i n e s   ------ */
        // DRIVE to lift Arm up
        //  Robot must be stopped for this to run
        //  Drive all the way to top before resetting the encoder with the
        //    left_bumper
        if (gamepad1.left_trigger > 0 
              && robot.isStopped(drive, turn, strafe)) {
            robot.liftMotor.setPower(gamepad1.left_trigger);
        }
        
        // Reset the lift motor encoder
        //   Robot must be stopped
        //   Arm should be all the way up
        if (gamepad1.left_bumper 
        && robot.isStopped(drive, turn, strafe)) {
            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        
        if (gamepad2.left_bumper) {
            robot.shooterOn();
        } else {
           if (!shooting) {
               robot.shooterOff();
           }
        }
        
        // reset keyPress status
        rightBumper2Press = gamepad2.right_bumper;
        


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Lift Encoder", robot.liftMotor.getCurrentPosition());

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stopRobot();

    }


    void shootARing() {
        // Turn Motors on
        robot.shooterOn();

        // set a timer
        shootCycle.reset();
        shooting = true;

        // start the servo
        robot.flipperServo.setPosition(1);
        for (int i=0; i<500; i++ ){
            telemetry.addData("Counter", i);
        }

        //pull the servo back
        robot.flipperServo.setPosition(0);
        for (int i=0; i<500; i++ ){
            telemetry.addData("Counter", i);
        }

    }

    
}
