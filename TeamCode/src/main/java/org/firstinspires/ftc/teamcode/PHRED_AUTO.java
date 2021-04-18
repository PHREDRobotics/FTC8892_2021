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

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.PHRED_Bot;

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
    private PHRED_Bot robot = new PHRED_Bot();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime fireTime = new ElapsedTime();

    private int FIRE_TIME = 2000;
    //drive motors
    /*private DcMotor  frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    //shooter motors
    private DcMotor frontShooterMotor = null;
    private DcMotor backShooterMotor = null;
    private Servo flipperServo = null;
*/
    private double FLIPPER_FORWARD = 1.5;
    private double FLIPPER_BACK = 0;
    //OM motors
    //private DcMotor tilter = null;
    private int TILTER_DOWN = -100;
    //private Servo grabberServo = null;
    // Sensors
    //public DistanceSensor frontRangeSensor = null;
    //public DistanceSensor rightRangeSensor = null;
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
        
                robot.initializeRobot(hardwareMap);

        
        /*// hardware maps
        //drive motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
        //shooter motors
        frontShooterMotor = hardwareMap.get(DcMotor.class, "front_shooter_motor");
        backShooterMotor = hardwareMap.get(DcMotor.class, "back_shooter_motor");
        flipperServo = hardwareMap.get(Servo.class, "flipper_servo");
        //OM motors
        tilter = hardwareMap.get(DcMotor.class, "lift_motor");
        grabberServo = hardwareMap.get(Servo.class, "gripper_servo");
        // Sensors
        frontRangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "front_range_sensor");
        //frontRange.setI2cAddress(I2cAddr.create8bit(0x3a));
        rightRangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "right_range_sensor");
        //rightRange.setI2cAddress(I2cAddr.create8bit(0x3c));
        //leftRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range_sensor");
        //leftRange.setI2cAddress(I2cAddr.create8bit(0x3e));
        *///gyro
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
        //backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        //backRightDrive.setDirection(DcMotor.Direction.REVERSE);


        //setup-------------------------------
            imu.initialize(parameters);

            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);

                idle();
            }
            if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }
           
        //end of setup-----------------------
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        
         //Site picker to drive into either A B or C
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0){
                        telemetry.addData("Objects:", "none" );
                        siteA();
                    }else{
                        int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                      
                    
                        if (recognition.getLabel().equals("Single")){
                            telemetry.addData("Objects:", "One");
                            siteB();
                        }else if (recognition.getLabel().equals("Quad")){
                            telemetry.addData("Objects:", "four");
                            siteC();
                        }
                      }
                    
                    }       
                         
                    
                }
            }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
                
            telemetry.update();

            //=(
            //List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
             //for (Recognition recognition : updatedRecognitions) {
                //telemetry.addData("label (%d)", recognition.getLabel());
             //}
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
        
        telemetry.addData("Vuforia init:","Complete" );
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        telemetry.addData("Tfod init:","Complete" );
    }
    public void siteA() {
        //using the front range sensor as reference it drives until it is at target spot
        while (robot.frontRangeSensor.getDistance(DistanceUnit.CM) >= 80 && !isStopRequested()){
            //keeps it no more than 30 cm from wall
            if (robot.rightRangeSensor.getDistance(DistanceUnit.CM) <= 25){
                robot.driveLeft(.5);
            }
            else {
                robot.driveForward(.8);
            }
            telemetry.addLine("Going to A");
              telemetry.addData("right range: %d", robot.rightRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("front range: %d", robot.frontRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
        //all three programs differ to turnAndDrop
        turnAndDrop(0, 0.5);
    }

    public void siteB() {
        while (robot.frontRangeSensor.getDistance(DistanceUnit.CM) >= 80 && !isStopRequested()){

            if (robot.rightRangeSensor.getDistance(DistanceUnit.CM) <= 25){
                robot.driveLeft(.5);
            }
            else {
                robot.driveForward(.8);
            }
            telemetry.addLine("Going to B");
            telemetry.addData("right range: %d", robot.rightRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("front range: %d", robot.frontRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
        turnAndDrop(-90, -.5);
    }

    public void siteC() {
        while (robot.frontRangeSensor.getDistance(DistanceUnit.CM) >= 80 && !isStopRequested()){

            if (robot.rightRangeSensor.getDistance(DistanceUnit.CM) <= 25){
                robot.driveLeft(.5);
            }
            else {
                robot.driveForward(.8);
            }
            telemetry.addLine("Going to C");
            telemetry.addData("right range: %d", robot.rightRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("front range: %d", robot.frontRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
        turnAndDrop(-180, -.5);
    }
    
    public void turnAndDrop(double turnAngle, double turnSpeed) {
        while (robot.angle() <= (turnAngle - 5) || robot.angle() >= (turnAngle + 5) && !isStopRequested()) {
            robot.turnLeft(turnSpeed);
            telemetry.addData("target %f", turnAngle);
            telemetry.addData("angle %f", robot.angle());
            telemetry.addData("Turning:", "True");
            telemetry.update();
        }
        telemetry.addData("Turning:", "done");
        robot.stopRobot();
        //
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setTargetPosition(TILTER_DOWN);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(.5);
        while (robot.liftMotor.isBusy() && !isStopRequested()){
            telemetry.addData("angle %f", robot.angle());
            telemetry.update();
            sleep (2000);
            robot.liftMotor.setPower(0);
        }
        
        
        while (robot.angle() >= 5 || robot.angle() >= -5 && !isStopRequested()) {
            robot.turnRight(.5);
            telemetry.addData("Turning:", "True");
        }
        robot.stopRobot();
        //returnToLine(60,60,true,1);

    }
    public void returnToLine(double xPOS,double yPOS,boolean firePerm,double fireSpeed){

        int turnAmount = 25;
        // moves backwards towards the line
        while(robot.frontRangeSensor.getDistance(DistanceUnit.CM) <= yPOS && !isStopRequested()){
            robot.driveBackwards(.5);

        }
        //moves away from the wall to firing position
        while(robot.rightRangeSensor.getDistance(DistanceUnit.CM) <= xPOS && !isStopRequested()){
            robot.driveLeft(.5);
        }
        //resets the amount of time the bot spends shooting one disk
        fireTime.reset();
        while (firePerm && fireTime.milliseconds() != FIRE_TIME && !isStopRequested()){
            robot.frontShooterMotor.setPower(fireSpeed);
            robot.backShooterMotor.setPower(fireSpeed);
        }
        //reload
        robot.flipperServo.setPosition(FLIPPER_FORWARD);
        sleep(300);
        robot.flipperServo.setPosition(FLIPPER_BACK);


        //turning
        while (robot.angle() >= turnAmount && !isStopRequested()) {
            robot.turnLeft(.2);
        }
        turnAmount = turnAmount + 25;
        //second target
        fireTime.reset();
        while (firePerm && fireTime.milliseconds() != FIRE_TIME && !isStopRequested()){
            robot.frontShooterMotor.setPower(fireSpeed);
            robot.backShooterMotor.setPower(fireSpeed);
        }
        robot.flipperServo.setPosition(FLIPPER_FORWARD);
        sleep(300);
        robot.flipperServo.setPosition(FLIPPER_BACK);

        //turning
        while (robot.angle() >= turnAmount && !isStopRequested()) {
            robot.turnLeft(.2);
        }
        //third target
        fireTime.reset();
        while (firePerm && fireTime.milliseconds() != FIRE_TIME && !isStopRequested()){
            robot.frontShooterMotor.setPower(fireSpeed);
            robot.backShooterMotor.setPower(fireSpeed);
        }
        robot.flipperServo.setPosition(FLIPPER_FORWARD);
        sleep(300);
        robot.flipperServo.setPosition(FLIPPER_BACK);
    }
}
