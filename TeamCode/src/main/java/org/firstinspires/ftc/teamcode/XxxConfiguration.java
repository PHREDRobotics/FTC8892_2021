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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PHRED_Bot;

import java.util.List;

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
    private double liftPower =0;
    
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    
    private static final String VUFORIA_KEY = "Ac7M0vn/////AAABmVPF98+WT0Tmowj61HNTEdAWwuxfzVemTNllfnBqmizoY+o47bat1Z7pQRR7AGHP6dSVoUKVPrv2vtr1miYGqMQs5DwgadxKlhpHvMRywSOM10XWKQjJY1dMWa4lJs7/YAvesnmdlatc6CrE9jQ4E1CQLR2pM1rmdI9Ns8RgxmgoRDq0TEPnA5bqIf3WIsnLbjKHuUN5tl4WTfSiKl9Tc1ujEK1GZj3UoNBxgTGXkpscyNUmIk2brrwu9bB+xq2CTaQ9P1bGQ4PpiaRTMElfW47Rg8uos+qdKF2THIUprzvb/gbvT6RDMmfteetOz3Vpj8qgki67/RPRzIUt+dzMe1u+MdpY0crPJ54M+aA3hnE8";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
      /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.initializeRobot(hardwareMap);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        initVuforia();
        initTfod();
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
            robot.liftMotor.setTargetPosition(-20);
            liftPower = 1.0;
            
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor.setPower(liftPower);
        } else {
            if (gamepad1.dpad_down) {
                robot.liftMotor.setTargetPosition(-110);
                liftPower = -0.5;
                
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftMotor.setPower(liftPower);
            } else {
                if (!robot.liftMotor.isBusy()){
                    liftPower = 0;
                    robot.liftMotor.setPower(liftPower);
                }
            }
        }
        /*
        if (gamepad1.left_trigger > 0) {
            //robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftPower = -gamepad1.left_trigger;
            robot.liftMotor.setPower(liftPower);
        } else {
            if (gamepad1.right_trigger > 0) {
                //robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftPower = gamepad1.right_trigger;
                robot.liftMotor.setPower(liftPower);

            } else {
                liftPower = 0;
                robot.liftMotor.setPower(liftPower);
            }
        }
        */
        telemetry.addData("Lift Power", "power (%s)", liftPower);
        telemetry.addData("Lift Encoder", "position (%s)", robot.liftMotor.getCurrentPosition());
        
        // Servo Testing
        
        if(gamepad1.b) {
            robot.gripperServo.setPosition(robot.gripperServo.getPosition() + 0.1);
        }
        if(gamepad1.a) {
            robot.gripperServo.setPosition(robot.gripperServo.getPosition() - 0.1);
        }
        if(gamepad1.y) {
            robot.flipperServo.setPosition(robot.flipperServo.getPosition() + 0.1);
        }
        if(gamepad1.x) {
            robot.flipperServo.setPosition(robot.flipperServo.getPosition() - 0.1);
        }
        
        telemetry.addData("Servos Position", "gripper (%.2f), flipper (%.2f)", 
           robot.gripperServo.getPosition(),
           robot.flipperServo.getPosition());
        telemetry.addData("Servos Direction", "gripper %s, flipper %s",
           robot.gripperServo.getDirection(),
           robot.flipperServo.getDirection());
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == LABEL_FIRST_ELEMENT){
                            telemetry.addData("sees:","4 Rings" );
                        } else if (recognition.getLabel() == LABEL_SECOND_ELEMENT){
                            telemetry.addData("sees:","1 Rings" );
                        } else {
                            telemetry.addData("sees:","0 Rings" );
                        }
                    }
                }
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
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
