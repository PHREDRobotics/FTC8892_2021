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

import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class PHRED_Bot
{
    /* Public members. */
    //CONSTANTS --------------
    public double FLIPPER_FORWARD = 1.0;
    public double FLIPPER_BACK = 0.5;



    // Motors ---------------------------
    public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor liftMotor = null;
    public DcMotor frontShooterMotor = null;
    public DcMotor backShooterMotor = null;
    public Servo gripperServo = null;
    public Servo flipperServo = null;

    // Sensors --------------------------
    public ModernRoboticsI2cRangeSensor frontRange = null;
    public ModernRoboticsI2cRangeSensor rightRange = null;
    public ModernRoboticsI2cRangeSensor leftRange = null;
    public ModernRoboticsI2cGyro gyro = null;

    /* Local Members */

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public void PHRED_bot(){

    }

    /* Initialize standard Hardware interfaces */
    public void initializeRobot() {

        // Save reference to Hardware map
        HardwareMap hwMap = null;

        // Define and initialize motors
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_motor");
        rightRearDrive = hwMap.get(DcMotor.class, "right_rear_motor");
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_motor");
        leftRearDrive = hwMap.get(DcMotor.class, "left_rear_motor");
        liftMotor = hwMap.get(DcMotor.class, "lift_motor");
        frontShooterMotor = hwMap.get(DcMotor.class, "front_shooter_motor");
        backShooterMotor = hwMap.get(DcMotor.class, "back_shooter_motor");

        flipperServo = hwMap.get(Servo.class, "flipper_servo");
        gripperServo = hwMap.get(Servo.class, "gripper_servo");

        // Set motor direction
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set Motors to zero power
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);

        // Set-up Sensors
        frontRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "front_range_sensor");
        frontRange.setI2cAddress(I2cAddr.create8bit(0x3a));
        rightRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "right_range_sensor");
        rightRange.setI2cAddress(I2cAddr.create8bit(0x3c));
        leftRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "left_range_sensor");
        leftRange.setI2cAddress(I2cAddr.create8bit(0x3e));

        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

    } // end Init

    /* -------------------------- */
    /* ---- Common Functions ---- */

    /* Robot Drive Functions */
    public void driveRobot(
            double rightFrontPower,
            double leftFrontPower,
            double rightBackPower,
            double leftBackPower) {
        rightFrontDrive.setPower(rightFrontPower);
        leftFrontDrive.setPower(leftFrontPower);
        rightRearDrive.setPower(rightBackPower);
        leftRearDrive.setPower(leftBackPower);
    }

    public void driveForward (double forwardSpeed) {
        driveRobot (forwardSpeed, forwardSpeed, forwardSpeed, forwardSpeed);
    }

    public void driveBackwards (double backwardSpeed) {
        driveRobot (-backwardSpeed, -backwardSpeed, -backwardSpeed, -backwardSpeed);
    }

    public void driveLeft (double leftSpeed) {
        driveRobot (leftSpeed, -leftSpeed, -leftSpeed, leftSpeed);
    }

    public void driveRight (double rightSpeed) {
        driveRobot (-rightSpeed, rightSpeed, rightSpeed, -rightSpeed);
    }
 
    //turn is reversed
    public void  turnRight (int rightRotate) {
        calibrateGyro();
        while (gyro.getHeading() < rightRotate) {
            driveRobot(1, -1, 1, -1);
        }
        calibrateGyro();
    }
    public void  turnLeft (int leftRotate) {
        calibrateGyro();
        while (gyro.getHeading() < leftRotate) {
            driveRobot(-1, 1, -1, 1);
        }
        calibrateGyro();
    }

    public void stopRobot() {
        driveRobot(0, 0, 0, 0);
    }

    /* Gyro Functions */

    public void calibrateGyro() {
        gyro.calibrate();
    }

    public boolean isCalibrating() {
        return gyro.isCalibrating();
    }

    /* Object Manipulation Functions */

    public void tiltToPosition(int tiltTarget){
        liftMotor.setTargetPosition(tiltTarget);
        liftMotor.setPower(.3);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (liftMotor.isBusy()){
            
        }
        liftMotor.setPower(0);
    }
    


}

