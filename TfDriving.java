//Created by Team 12009: Terminal Ferocity

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TfDriving1.0", group="TeleOp")
public class TfDriving extends OpMode {

    // Declare OpMode members
    
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor midMotor;
    DcMotor armMotor;
    DcMotor relicMotor;
    
    Servo jewelServo;
    Servo leftArm;
    Servo rightArm;
    Servo relicServo1;
    Servo relicServo2;
    
    ColorSensor colorSensor;
    
    double left;
    double right;
    double bottom;
    
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        midMotor = hardwareMap.dcMotor.get("midMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        relicMotor = hardwareMap.dcMotor.get("relicMotor");
        jewelServo = hardwareMap.servo.get("jewelServo");
        relicServo1 = hardwareMap.servo.get("relicServo1");
        relicServo2 = hardwareMap.servo.get("relicServo2");
        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        relicServo1.setPosition(.7);
        relicServo2.setPosition(0);
        jewelServo.setPosition(1);
        rightArm.setPosition(.7);
        leftArm.setPosition(.3);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        
        //Gamepad 1
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        
        if (gamepad1.dpad_right) {
            midMotor.setPower(-0.5);
        }
        else if (gamepad1.dpad_left) {
            midMotor.setPower(0.5);
        }
        else {
            midMotor.setPower(0);
        }


        if (gamepad1.dpad_up) {
            leftMotor.setPower(-.5);
            rightMotor.setPower(-.5);
        }
        else if (gamepad1.dpad_down) {
            leftMotor.setPower(.5);
            rightMotor.setPower(.5);
        }
        else if (gamepad1.right_bumper) {
            leftMotor.setPower(-.5);
            rightMotor.setPower(-.5);
            midMotor.setPower(-1);
        }
        else if (gamepad1.left_bumper) {
            leftMotor.setPower(-.5);
            rightMotor.setPower(-.5);
            midMotor.setPower(1);
        }
        else {
            leftMotor.setPower(left);
            rightMotor.setPower(right);
        }
        

        //Gamepad 2
        if (gamepad2.left_trigger > 0) {
            relicMotor.setPower(-.5);
        } 
        else if (gamepad2.right_trigger > 0) {
            relicMotor.setPower(.5);
        } 
        else {
            relicMotor.setPower(0);
        }

        
        if (gamepad2.right_bumper) {
            relicServo1.setPosition(.7);
            relicServo2.setPosition(0);
        }

        
        if (gamepad2.left_bumper) {
            relicServo1.setPosition(0);
            relicServo2.setPosition(1);
        }

        
        if (gamepad2.y) {
            relicServo1.setPosition(0);
        }


        if (gamepad2.a) {
            jewelServo.setPosition(-1);
        } 
        else {
            jewelServo.setPosition(1);
        }

        
        if (gamepad2.b) {
            rightArm.setPosition(.3);
            leftArm.setPosition(.7);
        }

        
        if (gamepad2.x) {
            rightArm.setPosition(0);
            leftArm.setPosition(1);
        }

        
        if (gamepad2.dpad_up && (armMotor.getCurrentPosition() < 1440*8)) {
            armMotor.setPower(.5);
        } 
        else if (gamepad2.dpad_down){
            armMotor.setPower(-.5);
        } 
        else if (gamepad2.right_stick_button) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(1);
        } 
        else if (gamepad2.left_stick_button) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(-1);
        } 
        else {
            armMotor.setPower(0);
        }
    }

    @Override
    public void stop() {
    }
}


