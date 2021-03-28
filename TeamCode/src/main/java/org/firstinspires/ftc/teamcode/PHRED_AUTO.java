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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class PHRED_AUTO extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime fireTime = new ElapsedTime();

    private int FIRE_TIME = 2000;
    //drive motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    //shooter motors
    private DcMotor frontShooterMotor = null;
    private DcMotor backShooterMotor = null;
    private Servo flipperServo = null;

    private double FLIPPER_FORWARD = 1.5;
    private double FLIPPER_BACK = 0;
    //OM motors
    private DcMotor tilter = null;
    private int TILTER_DOWN = 25;
    private Servo graber = null;
    // Sensors
    public ModernRoboticsI2cRangeSensor frontRange = null;
    public ModernRoboticsI2cRangeSensor rightRange = null;
    public ModernRoboticsI2cRangeSensor leftRange = null;
    //Gyro
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double correction;
    //camera
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // hardware maps
        //drive motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        //shooter motors
        frontShooterMotor = hardwareMap.get(DcMotor.class, "front_shooter");
        backShooterMotor = hardwareMap.get(DcMotor.class, "back_shooter");
        flipperServo = hardwareMap.get(Servo.class, "flipper_servo");
        //OM motors
        tilter = hardwareMap.get(DcMotor.class, "front_left_drive");
        graber = hardwareMap.get(Servo.class, "grab_servo");
        // Sensors
        frontRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_range_sensor");
        frontRange.setI2cAddress(I2cAddr.create8bit(0x3a));
        rightRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_range_sensor");
        rightRange.setI2cAddress(I2cAddr.create8bit(0x3c));
        leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range_sensor");
        leftRange.setI2cAddress(I2cAddr.create8bit(0x3e));
        //gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        //camera

        telemetry.addData("Map:","complete");

        //
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            imu.initialize(parameters);

            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);

                idle();
            }
//TODO all this stuff
            /*Site picker to drive into either A B or C
            siteA();
            siteB();
            siteC();
            */
        }
    }
    public void siteA() {
        //using the front range sensor as reference it drives until it is at target spot
        while (frontRange.getDistance(DistanceUnit.CM) >= 152 && !isStopRequested()){
            //keeps it no more than 30 cm from wall
            if (rightRange.getDistance(DistanceUnit.CM) >= 30){
                frontLeftDrive.setPower(-.5);
                frontRightDrive.setPower(-.5);
                backLeftDrive.setPower(.5);
                backRightDrive.setPower(.5);
            }
            else {
                frontLeftDrive.setPower(1);
                frontRightDrive.setPower(1);
                backLeftDrive.setPower(1);
                backRightDrive.setPower(1);
            }
        }
        //all three programs differ to turnAndDrop
        turnAndDrop();
    }

    public void siteB() {
        while (frontRange.getDistance(DistanceUnit.CM) >= 92 && !isStopRequested()){

            if (rightRange.getDistance(DistanceUnit.CM) >= 92){
                frontLeftDrive.setPower(-.5);
                frontRightDrive.setPower(-.5);
                backLeftDrive.setPower(.5);
                backRightDrive.setPower(.5);
            }
            else {
                frontLeftDrive.setPower(1);
                frontRightDrive.setPower(1);
                backLeftDrive.setPower(1);
                backRightDrive.setPower(1);
            }
        }
        turnAndDrop();
    }

    public void siteC() {
        while (frontRange.getDistance(DistanceUnit.CM) >= 31 && !isStopRequested()){

            if (rightRange.getDistance(DistanceUnit.CM) >= 31){
                frontLeftDrive.setPower(-.5);
                frontRightDrive.setPower(-.5);
                backLeftDrive.setPower(.5);
                backRightDrive.setPower(.5);
            }
            else {
                frontLeftDrive.setPower(1);
                frontRightDrive.setPower(1);
                backLeftDrive.setPower(1);
                backRightDrive.setPower(1);
            }
        }
        turnAndDrop();
    }

    public void turnAndDrop(){
        while (globalAngle <= 90 && !isStopRequested()) {
            frontLeftDrive.setPower(-.5);
            frontRightDrive.setPower(.5);
            backLeftDrive.setPower(-.5);
            backRightDrive.setPower(.5);
        }
        tilter.setTargetPosition(TILTER_DOWN);
        tilter.setPower(.2);
        tilter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (tilter.isBusy() && !isStopRequested()){}
        while (globalAngle >= 0 && !isStopRequested()) {
            frontLeftDrive.setPower(.5);
            frontRightDrive.setPower(-.5);
            backLeftDrive.setPower(.5);
            backRightDrive.setPower(-.5);
        }
        returnToLine(60,60,true,1);

    }
    public void returnToLine(double xPOS,double yPOS,boolean firePerm,double fireSpeed){

        int turnAmount = 25;
        // moves backwards towards the line
        while(frontRange.getDistance(DistanceUnit.CM) <= yPOS && !isStopRequested()){
            frontLeftDrive.setPower(-.5);
            frontRightDrive.setPower(-.5);
            backLeftDrive.setPower(-.5);
            backRightDrive.setPower(.5);


        }
        //moves away from the wall to firing position
        while(rightRange.getDistance(DistanceUnit.CM) <= xPOS && !isStopRequested()){
            frontLeftDrive.setPower(.5);
            frontRightDrive.setPower(.5);
            backLeftDrive.setPower(-.5);
            backRightDrive.setPower(-.5);
        }
        //resets the amount of time the bot spends shooting one disk
        fireTime.reset();
        while (firePerm && fireTime.milliseconds() != FIRE_TIME && !isStopRequested()){
            frontShooterMotor.setPower(fireSpeed);
            backShooterMotor.setPower(fireSpeed);
        }
        //reload
        flipperServo.setPosition(FLIPPER_FORWARD);
        sleep(300);
        flipperServo.setPosition(FLIPPER_BACK);


        //turning
        while (globalAngle >= turnAmount && !isStopRequested()) {
            frontLeftDrive.setPower(.5);
            frontRightDrive.setPower(-.5);
            backLeftDrive.setPower(.5);
            backRightDrive.setPower(-.5);
        }
        turnAmount = turnAmount + 25;
        //second target
        fireTime.reset();
        while (firePerm && fireTime.milliseconds() != FIRE_TIME && !isStopRequested()){
            frontShooterMotor.setPower(fireSpeed);
            backShooterMotor.setPower(fireSpeed);
        }
        flipperServo.setPosition(FLIPPER_FORWARD);
        sleep(300);
        flipperServo.setPosition(FLIPPER_BACK);

        //turning
        while (globalAngle >= turnAmount && !isStopRequested()) {
            frontLeftDrive.setPower(.5);
            frontRightDrive.setPower(-.5);
            backLeftDrive.setPower(.5);
            backRightDrive.setPower(-.5);
        }
        //third target
        fireTime.reset();
        while (firePerm && fireTime.milliseconds() != FIRE_TIME && !isStopRequested()){
            frontShooterMotor.setPower(fireSpeed);
            backShooterMotor.setPower(fireSpeed);
        }
    }
}
