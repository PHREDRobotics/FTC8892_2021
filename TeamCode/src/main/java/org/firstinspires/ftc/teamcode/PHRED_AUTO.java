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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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

@Autonomous(name="PHRED_Auto", group="Linear Opmode")

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
    public DistanceSensor frontRange = null;
    public DistanceSensor rightRange = null;
   // public ModernRoboticsI2cRangeSensor leftRange = null;
    //Gyro
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    double correction;
    //camera
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY = "Ac7M0vn/////AAABmVPF98+WT0Tmowj61HNTEdAWwuxfzVemTNllfnBqmizoY+o47bat1Z7pQRR7AGHP6dSVoUKVPrv2vtr1miYGqMQs5DwgadxKlhpHvMRywSOM10XWKQjJY1dMWa4lJs7/YAvesnmdlatc6CrE9jQ4E1CQLR2pM1rmdI9Ns8RgxmgoRDq0TEPnA5bqIf3WIsnLbjKHuUN5tl4WTfSiKl9Tc1ujEK1GZj3UoNBxgTGXkpscyNUmIk2brrwu9bB+xq2CTaQ9P1bGQ4PpiaRTMElfW47Rg8uos+qdKF2THIUprzvb/gbvT6RDMmfteetOz3Vpj8qgki67/RPRzIUt+dzMe1u+MdpY0crPJ54M+aA3hnE8";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        
        // hardware maps
        //drive motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_front_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_front_motor");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "left_rear_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "right_rear_motor");
        //shooter motors
        frontShooterMotor = hardwareMap.get(DcMotor.class, "front_shooter_motor");
        backShooterMotor = hardwareMap.get(DcMotor.class, "rear_shooter_motor");
        flipperServo = hardwareMap.get(Servo.class, "flipper_servo");
        //OM motors
        tilter = hardwareMap.get(DcMotor.class, "lift_motor");
        graber = hardwareMap.get(Servo.class, "gripper_servo");
        // Sensors
        frontRange = hardwareMap.get(Rev2mDistanceSensor.class, "front_range_sensor");
        //frontRange.setI2cAddress(I2cAddr.create8bit(0x3a));
        rightRange = hardwareMap.get(Rev2mDistanceSensor.class, "right_range_sensor");
        //rightRange.setI2cAddress(I2cAddr.create8bit(0x3c));
        //leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range_sensor");
        //leftRange.setI2cAddress(I2cAddr.create8bit(0x3e));
        //gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        //camera
        initVuforia();
        initTfod();


        telemetry.addData("Map:","complete");

        //
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            imu.initialize(parameters);

            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);

                idle();
            }
            //Site picker to drive into either A B or C
            //if (tfod != null) {
                //List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                //if (updatedRecognitions != null) {
                    //for (Recognition recognition : updatedRecognitions) {
                       // if (recognition.getLabel() == LABEL_FIRST_ELEMENT){
                            siteC();
                        //} else if (recognition.getLabel() == LABEL_SECOND_ELEMENT){
                        //    siteB();
                       // } else {
                       //     siteA();
                       // }
                   // }
               // }
           // }

        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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
