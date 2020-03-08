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

package org.firstinspires.ftc.teamcode.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumCalculator;

import java.util.concurrent.TimeUnit;

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

@Autonomous(name="Skystone Red Simple X", group="Red")

public class SkystoneRedX extends OpMode
{// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Hardware robot = new Hardware();
    boolean rotated = false;
    int delay = 0;
    int stage = 1;
    int pos = 0;
    double timeOfNewStage;
    MecanumCalculator calculator = new MecanumCalculator( 0.000125, 0.00025, 2 );
    //private DcMotor leftFront, leftBack, rightFront, rightBack, slide, claw, arm;
    //SLIDE MOTOR
    // 1120 Ticks/rev
    // d = 3cm, r = 1.5cm, C = 3pi cm
    // Dist = ticks/1120 * 3pi
    // 32cm length
    // MAX ENCODER = (32/3pi * 1120) = 3802.7, 3802 ticks+
    //private GyroSensor gyro;
    //DcMotor[] drivetrain;
    //private CRServo found;




    //public Drivetrain drive;
    private void nextStage(){
        stage++;
        timeOfNewStage = runtime.time(TimeUnit.SECONDS);
        robot.odometryY.setMode( DcMotorEx.RunMode.STOP_AND_RESET_ENCODER );
        robot.odometryX.setMode( DcMotorEx.RunMode.STOP_AND_RESET_ENCODER );
        robot.odometryY.setMode( DcMotorEx.RunMode.RUN_WITHOUT_ENCODER );
        robot.odometryX.setMode( DcMotorEx.RunMode.RUN_WITHOUT_ENCODER );
        robot.setMotorPowers(0,0,0,0);
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init( hardwareMap, telemetry );
        telemetry.addData("Status", "Initialized" );


        // create a sound parameter that holds the desired player parameters.

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        //drive = Drivetrain.init( 0, 0, 0, Drivetrain.driveType.fourWheel );

        // Tell the driver that initialization is complete.

        //gyro = hardwareMap.get( GyroSensor.class, "gyro" );
        //gyro.calibrate();


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        robot.initLoop();
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
        // First, rotate the  robot to be parallel to the face of the block

        double t = runtime.time(TimeUnit.SECONDS);
        if( stage == 1 ){
            if( t < timeOfNewStage + 2 ){
                robot.armA.setPower( 0.50 );
                robot.armB.setPower( 0.50 );
            }else{
                robot.armA.setPower( 0 );
                robot.armB.setPower( 0 );
            }
            robot.clawA.setPosition( 0 );
            robot.clawB.setPosition( 0 );
            calculator.setTarget( -22500,0,0 );
            double[] out = calculator.getOutput( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() );
            robot.mecanumDrive( out[0], out[1], out[2] );
            if( calculator.getDoneOverall( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() ) ){
                nextStage();
            }
        }else if( stage == 2 ){
            if( robot.colorA.green() < 15 ){
                pos = 1;
            }else{
                pos = 3;
            }
            if( t > timeOfNewStage + 2 ){
                nextStage();
            }
        }else if( stage == 3 ){
            if( pos == 1 ){
                robot.clawA.setPosition(1);
                robot.mecanumDrive(-0.2,0,0);
                if( t > timeOfNewStage + 1 ){
                    nextStage();
                }
            }else if( pos == 2 ){
                calculator.setTarget( 0,-6872,0 );
                calculator.setP(  0.0002, 0.0002, 2 );
                // calculator.setP( 0.0002, 0.004, 2 );
                double[] out = calculator.getOutput( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() );
                robot.mecanumDrive( out[0], out[1], out[2] );
                if( calculator.getDoneOverall( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() ) ){
                    nextStage();
                }
            }else if( pos == 3 ){
                robot.clawB.setPosition(1);
                robot.mecanumDrive(-0.2,0,0);
                if( t > timeOfNewStage + 1 ){
                    nextStage();
                }
            }
        }else if( stage == 4 ){
            if( pos == 1 ){
                nextStage();
            }else if( pos == 2 ){
                robot.clawB.setPosition(1);
                if( t > timeOfNewStage + 1 ){
                    nextStage();
                }
            }else if( pos == 3 ){
                nextStage();
            }
        }else if( stage == 5 ){
            // Back up
            calculator.setP( 0.0001, 0.0005, 2 );
            calculator.setTarget( 8600,0,0 );
            double[] out = calculator.getOutput( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() );
            robot.mecanumDrive( out[0], out[1], out[2] );
            if( calculator.getDoneOverall( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() ) ){
                nextStage();
            }
        }else if( stage == 6 ){
            // Go across
            calculator.setP(  0.000125, 0.0001, 2 );
            // 50000
            calculator.setTarget( 0,-40000,0 );
            double[] out = calculator.getOutput( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() );
            robot.mecanumDrive( out[0], out[1], out[2] );
            if( calculator.getDoneOverall( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() ) ){
                robot.clawA.setPosition( 0 );
                robot.clawB.setPosition( 0 );
                nextStage();
            }
        }else if( stage == 7 ){
            robot.odometryY.setPower(-1);
            if( t > timeOfNewStage + 2 ){
                robot.odometryY.setPower(0);
                nextStage();
            }
        }
        double[] error = calculator.getError( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() );
        boolean[] dones = calculator.getDoneIndiv( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() );
        boolean totalDone = calculator.getDoneOverall( robot.odometryX.getCurrentPosition(), robot.odometryY.getCurrentPosition(), robot.yaw() );
        //telemetry.addData("Slide Enc",robot.slide.getCurrentPosition() );
        telemetry.addData("Stage", stage );
        telemetry.addData( "SenseA", robot.colorA.green() );
        telemetry.addData( "SenseB", robot.colorB.green() );
        telemetry.addData( "Pos", pos );
        telemetry.addData( "At Target", totalDone );
        telemetry.addData( "Dones", dones[0] + " " + dones[1] + " " + dones[2] );
        telemetry.addData( "Odo X", error[0] );
        telemetry.addData( "Odo Y", error[1] );
        telemetry.addData( "Yaw", Math.toDegrees( robot.yaw() ) );
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
