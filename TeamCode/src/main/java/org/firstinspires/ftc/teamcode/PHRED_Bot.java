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
import com.qualcomm.robotcore.util.Hardware;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
    public double FLIPPER_SERVO_FORWARD = 1.0;
    public double FLIPPER_SERVO_BACK = 0.5;
    public double GRIPPER_SERVO_FORWARD = 1.0;
    public double GRIPPER_SERVO_BACK = 0.5;

    public int LIFT_TOP= 25;
    public int LIFT_BOTTOM = 0;
    public double LIFT_SPEED_MAX = 0.1;

    public double SHOOTING_SPEED = 1.0;

    // Motors ---------------------------
    public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor liftMotor = null;
    public DcMotor frontShooterMotor = null;
    public DcMotor rearShooterMotor = null;
    public Servo gripperServo = null;
    public Servo flipperServo = null;

    // Sensors --------------------------
    public Rev2mDistanceSensor frontRange = null;

    /* Local Members */
    HardwareMap hardwareMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public void PHRED_bot(){

    }

    /* Initialize standard Hardware interfaces */
    public void initializeRobot(HardwareMap anHwMap) {
        
        hardwareMap = anHwMap;

        // Define and initialize motors
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_motor");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_motor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_motor");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_motor");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        frontShooterMotor = hardwareMap.get(DcMotor.class, "front_shooter_motor");
        rearShooterMotor = hardwareMap.get(DcMotor.class, "rear_shooter_motor");

        flipperServo = hardwareMap.get(Servo.class, "flipper_servo");
        gripperServo = hardwareMap.get(Servo.class, "gripper_servo");

        // Set motor direction
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        
        frontShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        rearShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        

        // Set Motors to zero power
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);

        // Set-up Sensors
        // frontRange = hwMap.get(Rev2mDistanceSensor.class, "front_range_sensor");

        ////  gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

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
    
    public void shooterOn() {
        frontShooterMotor.setPower(SHOOTING_SPEED);
        rearShooterMotor.setPower(SHOOTING_SPEED);
    }
    
    public void shooterOff() {
        frontShooterMotor.setPower(0.0);
        rearShooterMotor.setPower(0.0);
    }
/* 
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
*/
    public void stopRobot() {
        driveRobot(0, 0, 0, 0);
    }

    /* Gyro Functions */
/*
    public void calibrateGyro() {
        gyro.calibrate();
    }

    public boolean isCalibrating() {
        return gyro.isCalibrating();
    }
*/
    /* Object Manipulation Functions */
/*
    public void tiltToPosition(int tiltTarget){
        liftMotor.setTargetPosition(tiltTarget);
        liftMotor.setPower(.3);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (liftMotor.isBusy()){
            
        }
        liftMotor.setPower(0);
    }
   */ 


}

