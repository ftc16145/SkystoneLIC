package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


import java.util.ArrayList;
/*
Fortnite!
* ⠀⠀⠀⠀⠀⠀⣤⣿⣿⠶⠀⠀⣀⣀

⠀⠀⠀⠀⠀⠀⣀⣀⣤⣤⣶⣿⣿⣿⣿⣿⣿

⠀⠀⣀⣶⣤⣤⠿⠶⠿⠿⠿⣿⣿⣿⣉⣿⣿

⠿⣉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠛⣤⣿⣿⣿⣀

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⣿⣿⣿⣿⣶⣤

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣤⣿⣿⣿⣿⠿⣛⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⠛⣿⣿⣿⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣶⣿⣿⠿⠀⣿⣿⣿⠛

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⠀⠀⣿⣿⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠿⠿⣿⠀⠀⣿⣶

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠛⠀⠀⣿⣿⣶

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⣿⣿⠤

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠿⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣀

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣶⣿
* */
public class Hardware {

    // Timer
        private ElapsedTime runtime = new ElapsedTime();
    // Scoring Mechanisms
        public DcMotorEx leftFront, leftBack, rightFront, rightBack;
        public ArrayList<DcMotorEx> drivetrain = new ArrayList<DcMotorEx>();
        double prevXPower, prevYPower;

        public DcMotorEx odometryX, odometryY;

        public Servo found = null;
        public Servo cap = null;

        public DcMotorEx armA, armB;
        public Servo clawA, clawB;

        public ColorSensor lineColor, colorA, colorB;


        public BNO055IMU imu = null;

        double targetAng = 0;
        boolean lastHit = true;
        boolean firstCall = true;
    // Info for the class
        HardwareMap hwMap = null;
        Telemetry tel = null;



        /*
        * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
        * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
        * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
        * web site at https://developer.vuforia.com/license-manager.
        *
        * Vuforia license keys are always 380 characters long, and look as if they contain mostly
        * random data. As an example, here is a example of a fragment of a valid key:
        *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
        * Once you've obtained a license key, copy the string from the Vuforia web site
        * and paste it in to your code on the next line, between the double quotes.
        */





        Context myApp;



    // Sounds
        String sounds[] = {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie"};
        boolean soundPlaying = false;
        int soundIndex, soundID;
        SoundPlayer.PlaySoundParams params;





    public void playSound(String sound) {
        if (!soundPlaying) {
            if ((soundID = myApp.getResources().getIdentifier(sound, "raw", myApp.getPackageName())) != 0) {

                // Signal that the sound is now playing.
                soundPlaying = true;

                // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
                SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                        new Runnable() {
                            public void run() {
                                soundPlaying = false;
                            }
                        });
            }
        }
    }

    public void setDriveModes( DcMotorEx.RunMode r ){
        for( DcMotorEx m : drivetrain ){
            m.setMode(r);
        }
    }

    public void resetDriveMode( DcMotorEx.RunMode r ){
        setDriveModes( DcMotorEx.RunMode.STOP_AND_RESET_ENCODER );
        setDriveModes( r );
    }

    public void init( HardwareMap hardware, Telemetry atel ){
        hwMap = hardware;
        tel = atel;

        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        drivetrain.add(leftFront);
        drivetrain.add(leftBack);
        drivetrain.add(rightFront);
        drivetrain.add(rightBack);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        for( DcMotorEx m : drivetrain ){
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        odometryX = hwMap.get( DcMotorEx.class, "odomX" );
        odometryY = hwMap.get( DcMotorEx.class, "odomY/Tape" );

        odometryX.setMode( DcMotorEx.RunMode.STOP_AND_RESET_ENCODER );
        odometryX.setMode( DcMotorEx.RunMode.RUN_WITHOUT_ENCODER );

        odometryY.setMode( DcMotorEx.RunMode.STOP_AND_RESET_ENCODER );
        odometryY.setMode( DcMotorEx.RunMode.RUN_WITHOUT_ENCODER );
        odometryY.setDirection( DcMotorEx.Direction.REVERSE );

        found = hwMap.get( Servo.class, "foundation" );
        found.setDirection(Servo.Direction.REVERSE);
        cap = hwMap.get( Servo.class, "cap");

        armA = hwMap.get( DcMotorEx.class, "armA" );
        armA.setDirection(DcMotorEx.Direction.REVERSE);
        armB = hwMap.get( DcMotorEx.class, "armB" );
//        armC = hwMap.get( DcMotorEx.class, "armC" );

        clawA = hwMap.get( Servo.class, "clawA" );
        clawB = hwMap.get( Servo.class, "clawB" );
        clawB.setDirection(Servo.Direction.REVERSE);
//        clawC = hwMap.get( Servo.class, "clawC" );

        cap.setDirection(Servo.Direction.REVERSE);

        lineColor = hwMap.get( ColorSensor.class, "color" );

        colorA = hwMap.get( ColorSensor.class, "senseA" );
        colorB = hwMap.get( ColorSensor.class, "senseB" );


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        // Random Sound/Vision Things
        soundIndex = 0;
        soundID = -1;
        myApp = hwMap.appContext;
        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
        playSound("ss_light_saber");
    }

    public void initLoop(){

    }

    public void setMotorPowers( double lf, double lb, double rf, double rb ){
        leftFront.setPower( lf );
        rightFront.setPower( rf );
        leftBack.setPower( lb );
        rightBack.setPower( rb );
    }

    public double yaw(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
    }
    public void hardBrake(){
        setMotorPowers(0,0,0,0);
    }


    public void mecanumDrive(double x, double y, double rot) {
        double nX, nY;
        // Updated from 0.1 to 0.2, testing to amp up accel
        nX = (Math.abs(x - prevXPower) > 0.25) ? prevXPower + Math.signum(x - prevXPower) * 0.25 : x;
        nY = (Math.abs(y - prevYPower) > 0.25) ? prevYPower + Math.signum(y - prevYPower) * 0.25 : y;

        double r = Math.hypot(-nX, nY);
        double robotAngle = Math.atan2(nY, -nX) - Math.PI / 4;
        double rightX = rot;
        double v1 = r * Math.cos(robotAngle);
        double v2 = r * Math.sin(robotAngle);
        double v3 = r * Math.sin(robotAngle);
        double v4 = r * Math.cos(robotAngle);
        double[] vals = {v1,v2,v3,v4};

        // Calculate maximum out of the 4 values
        double max = 0;
        for( double v : vals ){
            if( Math.abs( v ) > max ){
                max = Math.abs( v );
            }
        }

        // As long as the robot is in fact moving and is not just turning
        // Set all values to at most 1, mult by r to account for the magnitude input

        // Might need to make these deadbands instead of a hard zero
        if( max != 0 ){
            v1 = ( ( v1 / max ) * r );
            v2 = ( ( v2 / max ) * r );
            v3 = ( ( v3 / max ) * r );
            v4 = ( ( v4 / max ) * r );
        }
        v1 -= rightX;
        v2 -= rightX;
        v3 += rightX;
        v4 += rightX;
        setMotorPowers(v1,v2,v3,v4);
        // NOW: leftFront, leftBack, rightFront, rightBack
        prevXPower = nX;
        prevYPower = nY;

    }

    public void mecanumDriveFieldOrient( double x, double y, double rot, double zeroAng ){
        double adjustAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + zeroAng;
        double newX = Math.cos(adjustAngle) * x - Math.sin(adjustAngle) * y;
        double newY = Math.sin(adjustAngle) * x + Math.cos(adjustAngle) * y;
        mecanumDrive(newX, newY, rot);
    }

    public void mecanumDriveFieldOrient(double x, double y, double rot) {
        mecanumDriveFieldOrient( x, y, rot, 0 );
    }

    public void foundationControls( boolean up, boolean down ) {
        if( down ){
            lastHit = false;
        }else if( up ){
            lastHit = true;
        }
        found.setPosition( lastHit ? 0 : 1 );
    }

    public void foundationControls( boolean up ){
        lastHit = up;
        found.setPosition( lastHit ? 0 : 1 );
    }








}
