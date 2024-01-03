package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "free")


public class Autonomus extends LinearOpMode {

    private DcMotorEx wheelFL;
    private DcMotorEx wheelFR;
    private DcMotorEx wheelBL;
    private DcMotorEx wheelBR;
    private DcMotorEx Viper;
    private DcMotorEx in;
    private DcMotorEx climb;


    private Servo flip;
    //private DcMotorEx Insertnamehere
    //private DcMotorEx Insertnamehere
    private Servo drone;
    private Servo claw;


    @Override
    public void runOpMode() throws InterruptedException {
        //Motors
        wheelFL = hardwareMap.get(DcMotorEx.class, "wheelFL");
        wheelFR = hardwareMap.get(DcMotorEx.class, "wheelFR");
        wheelBL = hardwareMap.get(DcMotorEx.class, "wheelBL");
        wheelBR = hardwareMap.get(DcMotorEx.class, "wheelBR");


        Viper = hardwareMap.get(DcMotorEx.class, "viper");
        in = hardwareMap.get(DcMotorEx.class, "in");
        climb = hardwareMap.get(DcMotorEx.class, "climb");

        //------------SERVOS////
        claw = hardwareMap.get(Servo.class, "claw");
        flip = hardwareMap.get(Servo.class, "flip");
        drone = hardwareMap.get(Servo.class, "drone");

        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE
        wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBL.setDirection(DcMotorSimple.Direction.FORWARD);//FORWARD
        wheelBR.setDirection(DcMotorSimple.Direction.REVERSE);//REVERSE


        waitForStart();

        moveForward(1, 1000);
        moveBackward(-1, 1000);
    }
    public void moveForward (int power, int time ){
        wheelFL.setPower(power);
        wheelBL.setPower(power);
        wheelBR.setPower(power);
        wheelBL.setPower(power);
        sleep(time);
        wheelFL.setPower(0);
        wheelFR.setPower(0);
        wheelBL.setPower(0);
        wheelBR.setPower(0);


    }
    public  void  moveRight (int power, int time , int power2) {
        wheelFL.setPower(power);
        wheelFR.setPower(power2);
        wheelBL.setPower(power);
        wheelBR.setPower(power2);
        sleep(time);
        wheelFL.setPower(0);
        wheelFR.setPower(0);
        wheelBL.setPower(0);
        wheelBR.setPower(0);

    }
    public  void moveLeft (int power, int time , int power2) {
        wheelFL.setPower(power2);
        wheelFR.setPower(power);
        wheelBL.setPower(power2);
        wheelBR.setPower(power);
        sleep(time);
        wheelFL.setPower(0);
        wheelFR.setPower(0);
        wheelBL.setPower(0);
        wheelBR.setPower(0);

    }
    public  void  moveBackward(int power, int time ) {
        wheelFL.setPower(power);
        wheelFR.setPower(power);
        wheelBL.setPower(power);
        wheelBR.setPower(power);
        sleep(time);
        wheelFL.setPower(0);
        wheelFR.setPower(0);
        wheelBL.setPower(0);
        wheelBR.setPower(0);

    }






}
