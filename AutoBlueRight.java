package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorManager;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
 
@Autonomous(name="autoBlueRight", group ="Autonomous")
public class AutoBlueRight extends LinearOpMode {
    
    /* Declare OpMode members. */
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor midMotor;
    DcMotor armMotor;
    
    Servo jewelServo;
    Servo leftArm;
    Servo rightArm;
    Servo relicServo1;
    Servo relicServo2;
    
    phoneSensor gyro;
    phoneSensor accel;
    
    ColorSensor colorSensor;
    
    double left;
    double right;
    double bottom1;
    double angle = 1;
    double deltaAngle = 1; 
    double phase = 1;
    double column = 1;
    
    boolean colorDetected;
    
    static final double ENCODER_CPR    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double GEAR_RATIO    = 1 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = (ENCODER_CPR * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;

    public static final String TAG = "Vuforia VuMark Sample";
    
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    
    @Override public void runOpMode() throws InterruptedException {
        
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        midMotor = hardwareMap.dcMotor.get("midMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        jewelServo = hardwareMap.servo.get("jewelServo");
        relicServo1 = hardwareMap.servo.get("relicServo1");
        relicServo2 = hardwareMap.servo.get("relicServo2");
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        relicServo1.setPosition(.7);
        relicServo2.setPosition(0);
        jewelServo.setPosition(1);
        rightArm.setPosition(.7);
        leftArm.setPosition(.3);
    
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AXWZcLL/////AAAAGYCcsRtBUkNxiyu+Ou2RGpFDCqMl4iCECw3Qy/FOuiFNCZXzyruu03x1guQYrlkaopplIvPIL65vlGHAyu2NJsfP9DnpqJUSVvuxUOPUfaeFpu2PCulZ7xaOmfdAyF1JSjGDhdz8h01EUB2Surp5vHhqqVhkuBCCui6Vf8Eyy9E6wa5Fs2Y+cInVC/5FEw/+xNYbeZ9aJu8qE1iqU/b78Kzcha3m8FXRsnZl8XYBzGUGndE5A/aT9t+vgcTi8YaBLNvrGOQg+HjVFZIErRJFRgxlZNTr0I6/1BGG8CUg9Ef0f3FjuDoeoTs1BhtPV/jJ8frgQu5YSB9bWxKe91Z1EiNxp5TwzPIt21nAdZlGVNzc";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        this.init(hardwareMap);
        
        waitForStart();

        relicTrackables.activate();
        rightArm.setPosition(0);
        leftArm.setPosition(1);
        
        long startTime = System.currentTimeMillis();
        jewelServo.setPosition(0);
        armDistance(.5, 1);

        while (opModeIsActive()) {
        
            if (phase == 1) {
                
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        
                telemetry.addData("column", column);
                telemetry.update();
                
                if (System.currentTimeMillis() - startTime < 10000) {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        column = 1;
                        phase = 2;
                    }
                    if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        column = 3;
                        phase = 2;
                    }
                    if (vuMark == RelicRecoveryVuMark.CENTER) {
                        column = 2;
                        phase = 2;
                    }
                    if (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                    }
                }
                else {
                    phase = 2;    
                }
            }
            
            if (phase == 2) {
    
                telemetry.addData("red", colorSensor.red());
                telemetry.addData("blue", colorSensor.blue());
                telemetry.addData("column", column);
                telemetry.update();
                
                if (System.currentTimeMillis() - startTime < 20000 || colorDetected) {
                    
                    if (colorSensor.red() > colorSensor.blue()) {
                        colorDetected = true;
                        driveForwardDistance(.25, 4);
                        jewelServo.setPosition(1);
                        phase = 3;
                    }
                    else if (colorSensor.blue() > colorSensor.red()) {
                        colorDetected = true;
                        turnLeft(10);
                        jewelServo.setPosition(1);
                        sleep(1000);
                        turnRight(-10);
                        driveForwardDistance(.25, 4);
                        phase = 3;
                    }
                    else {
                        colorDetected = false;
                    }
                }
                else {
                    phase = 3;
                }
            }
            
            if (phase == 3) {
                
                telemetry.addData("column", column);
                telemetry.update();
                
                driveForwardDistance(0.5, (23.8 + ((column-1)*7.63)));
                turnLeft(90);
                
                driveForwardDistance(0.5, 10);
                rightArm.setPosition(.7);
                leftArm.setPosition(.3);
                driveForwardDistance(-0.5, -8);
                armDistance(-.5, -1);
                phase = 4;
                
            }
            
            if (phase == 4) {
            }
        }
    }
    
    public void init(HardwareMap hwMap) {
        gyro = new phoneSensor(Sensor.TYPE_GYROSCOPE, SensorManager.SENSOR_DELAY_GAME, hwMap);
        accel = new phoneSensor(Sensor.TYPE_ACCELEROMETER, SensorManager.SENSOR_DELAY_GAME, hwMap);
    }
    
    public void driveForward(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    
    public void armDistance(double power, double rotations) {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition((int)rotations*1440);
        armMotor.setPower(power);
        while (armMotor.isBusy()) {}
        armMotor.setPower(0);
    }
    
    public void driveForwardDistance(double power, double inches) {

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ROTATIONS = inches/CIRCUMFERENCE;
        double counts = ENCODER_CPR * GEAR_RATIO * ROTATIONS;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition((int) counts);
        rightMotor.setTargetPosition((int) counts);

        driveForward(power);


        while (leftMotor.isBusy() && rightMotor.isBusy()) {}


        driveForward(0);
    }
    
    public void turnLeft(double deg) throws InterruptedException {
        
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left = 0;
        angle = 0;

        while(left == 0){

            deltaAngle = gyro.values[0] * 1/50 * (180/Math.PI);

            angle = angle + deltaAngle;

            //Retrieve and display acc. data
            telemetry.addData("angle", angle);
            telemetry.update();

            Thread.sleep(20);

            if (angle < deg){
                rightMotor.setPower(.2);
                leftMotor.setPower(-.2);
            }

            else {
                rightMotor.setPower(0);
                leftMotor.setPower(0);
                left = 1;
            }

        }
    }
    
    public void turnRight(double deg) throws InterruptedException {
        
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right = 0;
        angle = 0;

        while(right == 0){
            deltaAngle = gyro.values[0] * 1/50 * (180/Math.PI);

            angle = angle + deltaAngle;

            //Retrieve and display acc. data
            telemetry.addData("angle", angle);
            telemetry.update();

            Thread.sleep(20);

            if (angle > deg){
                rightMotor.setPower(-.2);
                leftMotor.setPower(.2);
            }

            else {
                rightMotor.setPower(0);
                leftMotor.setPower(0);
                right = 1;
            }
        }    
    }
    
    public void stopOpMode() {
        gyro.unregister();
    }
    
}

