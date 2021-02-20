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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PHRED_Bot;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TeleOp: Iterative OpMode", group = "Iterative Opmode")

public class TeleOpMode extends OpMode {

    PHRED_Bot robot = new PHRED_Bot();
    // Declare OpMode members.;
    private int TILT_TOO_MUCH = 419;
    private int TILT_TOO_LITTLE = -58;
    private int WINCH_TOO_LITTLE = -1800;
    private int WINCH_TOO_MUCH = 580;
    private double SERVO_OPEN = 1.8;
    private double SERVO_CLOSED = .45;
    private ElapsedTime runtime = new ElapsedTime();

    private boolean dpadUpReleased = true;
    private boolean dpadDownReleased = true;
    private boolean tiltIsMoving = false;

    private double rightFrontPower = 0.0;
    private double rightRearPower = 0.0;
    private double leftFrontPower = 0.0;
    private double leftRearPower = 0.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.initializeRobot();
        // Tell the driver that initialization is complete.

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.tilterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winchyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        robot.tilterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winchyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        int encoderT;
        int encoderW;

        // Mechanum Mode use Left Stick for motion and Right Stick to rotate
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        double winch = gamepad2.left_stick_y;
        //     double tilt = gamepad2.right_stick_y;
        // Calculate individual wheel power
        // to keep us from knocking the tower

        robot.tilterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.winchyMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // - This uses basic math to combine motions and is easier to drive straight.
        leftFrontPower = (drive + turn + strafe);
        rightFrontPower = (-drive + turn + strafe);
        leftRearPower = (-drive + -turn + strafe);
        rightRearPower = (drive + -turn + strafe);
        // Send calculated power to wheels
        telemetry.addData("unlimited_power:", leftFrontPower);
        telemetry.addData("unlimited_power2:", drive);
        telemetry.addData("unlimited_power3:", gamepad1.a);
        if (gamepad1.a) {
            /*leftFrontDrive.setPower(leftFrontPower*0.5);  
            rightFrontDrive.setPower(rightFrontPower*0.5);
            leftRearDrive.setPower(leftRearPower*0.5);
            rightRearDrive.setPower(rightRearPower*0.5);
            telemetry.addData("orca", leftFrontDrive.getPower());
        */
            robot.leftFrontDrive.setPower(-.25);
            robot.rightFrontDrive.setPower(-.25);
            robot.leftRearDrive.setPower(.25);
            robot.rightRearDrive.setPower(.25);
        } else {
            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftRearDrive.setPower(leftRearPower);
            robot.rightRearDrive.setPower(rightRearPower);
        }
        //OM stuff
        robot.winchyMotor.setPower(-winch * .5);
        encoderT = robot.tilterMotor.getCurrentPosition();
        encoderW = robot.winchyMotor.getCurrentPosition();
/*
        if (encoderW > WINCH_TOO_MUCH) {
            winchyMotor.setPower(1);

        } else {
            if (encoderW < WINCH_TOO_LITTLE) {
                winchyMotor.setPower(-1);

            } else {
                winchyMotor.setPower(-winch * .5);
            }
        }
 */
           /*
           if (encoderT < 0){
            winchyMotor.setPower(0);
                    }

        if (encoderT > TILT_TOO_MUCH) {
            tilterMotor.setPower(-1);
        } else {
            if (encoderT < TILT_TOO_LITTLE) {
                tilterMotor.setPower(1);
            } else {
                tilterMotor.setPower(-tilt * .5);
            }
        }
        */
        // DPad Up -> Start Tilting Lift up
        if (gamepad2.dpad_up && dpadUpReleased && !tiltIsMoving) {
            dpadUpReleased = false;
            robot.tilterMotor.setTargetPosition(TILT_TOO_MUCH);
            robot.tilterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.tilterMotor.setPower(.3);
            tiltIsMoving = true;
        } else {
            dpadUpReleased = true;
        }

        // DPad Down -> Start Tilting Lift Back
        if (gamepad2.dpad_down && dpadDownReleased) {
            dpadDownReleased = false;
            robot.tilterMotor.setTargetPosition(TILT_TOO_LITTLE);
            robot.tilterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.tilterMotor.setPower(-.3);
            tiltIsMoving = true;
        } else {
            dpadDownReleased = true;
        }

        // Tilting action done? -> stop the Tilt Motor
        if (!robot.tilterMotor.isBusy() && tiltIsMoving) {
            robot.tilterMotor.setPower(0);
            robot.tilterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            tiltIsMoving = false;
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Tilt Action", (tiltIsMoving ? "Moving" : "Stationary") )
        telemetry.addData("Front Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("EncoderT", encoderT);
        telemetry.addData("EncoderW", encoderW);
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        robot.winchyMotor.setPower(0);
        robot.tilterMotor.setPower(0);

    }


}
