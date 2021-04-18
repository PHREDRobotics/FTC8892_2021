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

import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.annotation.Target;
import java.util.List;


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

    public int LIFT_TOP= -40;
    public int LIFT_BOTTOM = -100;
    public double LIFT_SPEED_MAX = 0.7;

    public double SHOOTING_SPEED = 1.0;
    public int SHOOTING_MILLISECONDS = 2500;

    // Motors ---------------------------
    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor liftMotor = null;
    public DcMotor frontShooterMotor = null;
    public DcMotor backShooterMotor = null;
    public Servo gripperServo = null;
    public Servo flipperServo = null;

    // Sensors --------------------------
    public Rev2mDistanceSensor frontRangeSensor = null;
    public Rev2mDistanceSensor rightRangeSensor = null;
    
    // Inertial Measurement Unit - IMU ---
    public BNO055IMU imu;
    
    /* Local Members */
    HardwareMap hardwareMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public void PHRED_bot(){

    }

    /* Initialize standard Hardware interfaces */
    public void initializeRobot(HardwareMap anHwMap) {
        
        hardwareMap = anHwMap;

        // --   D e f i n e   a n d   I n i t i a l i z e   H a r d w a r e   --
        
        // Drive Motors
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        // ------ Set motor direction
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        // ------ Set Motors to zero power
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);

        // Shooter Motors
        frontShooterMotor = hardwareMap.get(DcMotor.class, "front_shooter_motor");
        backShooterMotor = hardwareMap.get(DcMotor.class, "back_shooter_motor");
        // ------ Set motor direction
        frontShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        backShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        // ------ Set Motors to zero power
        frontShooterMotor.setPower(0);
        backShooterMotor.setPower(0);

        // Object Manipulation Motors
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        // ------ Set motor direction
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        
        // Set up Servos
        flipperServo = hardwareMap.get(Servo.class, "flipper_servo");
        gripperServo = hardwareMap.get(Servo.class, "gripper_servo");
        
        // Range Sensors
        frontRangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "front_range_sensor");
        rightRangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "right_range_sensor");

        // Inertial Measurement Unit - IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

    } // end Init

    /* -------------------------- */
    /* ---- Common Functions ---- */

    /* Robot Drive Functions */
    public void driveRobot(
            double frontRightPower,
            double frontLeftPower,
            double backRightPower,
            double backLeftPower) {
        frontRightDrive.setPower(frontRightPower);
        frontLeftDrive.setPower(frontLeftPower);
        backRightDrive.setPower(backRightPower);
        backLeftDrive.setPower(backLeftPower);
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
        backShooterMotor.setPower(SHOOTING_SPEED);
    }
    
    public void shooterOff() {
        frontShooterMotor.setPower(0.0);
        backShooterMotor.setPower(0.0);
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
    
    public boolean isStopped(double drive, double turn, double strafe) {
        return (drive == 0.0 && turn == 0.0 && strafe == 0.0); 
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

