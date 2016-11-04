package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.net.LinkAddress;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.*;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Random;

/**
 * Created by Student on 10/18/2016.
 */
@Autonomous(name="Auto 1", group="Iterative Opmode")
public class FirstAuto extends LinearOpMode {
    //BEFORE YOU CHANGE THIS CODE, TEST IT.  THEN THINK ABOUT WHETHER OR NOT YOUR CHANGE WILL BREAK THE CODE
    //THEN DON'T CHANGE THE CODE
    public GyroSensor gyroSensor=null;
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor   = null;
    public ColorSensor colorSensor=null;
    public Servo buttonPresser=null;
    String MYCOLOR="red";
    public void turnRightDegrees(double X) {
        float startVal = gyroSensor.getHeading();
        float goal = (float) (startVal + X);
        if (goal > 360) {
            goal -= 360;
        }
        if (goal < 0) {
            goal += 360;
        }
        while(!(Math.abs(gyroSensor.getHeading()-goal) < 18 || Math.abs(gyroSensor.getHeading()-goal+360) < 18 || Math.abs(gyroSensor.getHeading()-goal-360) < 18)&& opModeIsActive()){
            leftMotor.setPower(-.2);
            rightMotor.setPower(+.2);
            telemetry.addData("current angle: ",gyroSensor.getHeading());
            telemetry.addData("goal: ",goal);
            telemetry.addData("Fast",true);
            telemetry.update();
        }
        while(Math.abs(gyroSensor.getHeading()-goal) > 1  && opModeIsActive()){
            leftMotor.setPower(-.07);
            rightMotor.setPower(+.07);
            telemetry.addData("current angle: ",gyroSensor.getHeading());
            telemetry.addData("Slow",true);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void turnLeftDegrees(double X){
        //turn left
        float startVal = gyroSensor.getHeading();
        float goal = (float) (startVal - X);
        if (goal > 360) {
            goal -= 360;
        }
        if (goal < 0) {
            goal += 360;
        }
        while(!(Math.abs(gyroSensor.getHeading()-goal) < 18 || Math.abs(gyroSensor.getHeading()-goal+360) < 18 || Math.abs(gyroSensor.getHeading()-goal-360) < 18)&& opModeIsActive()){
            leftMotor.setPower(.2);
            rightMotor.setPower(-.2);
            telemetry.addData("current angle: ",gyroSensor.getHeading());
            telemetry.addData("goal: ",goal);
            telemetry.addData("Fast",true);
            telemetry.update();
        }
        while(Math.abs(gyroSensor.getHeading()-goal) > 1  && opModeIsActive()){
            leftMotor.setPower(.07);
            rightMotor.setPower(-.07);
            telemetry.addData("current angle: ",gyroSensor.getHeading());
            telemetry.addData("Slow",true);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }


    public void travelMeters(float d) {
        travelMeters(d, 0.3f);
    }
    public void travelMeters(float d, float power) {
        d*=100;
        //convert d to cm
        int correction = -400;
        float distancePerTick=(10000+correction)/218;//ticks/cm -- experimentally determined
        if(d > 0) {
            encoderDriveWithRamp(d*distancePerTick, power);
        } else {
            //encoderDriveWithRamp(-d*distancePerTick, power);
            encoderDriveWithRamp(-d*distancePerTick, power);
        }
    }
    public void encoderDriveWithRamp(float distance, float maxPower){
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetEncoders();
        float power=0;
        float rampTime=2;//seconds
        //p(id) loop, telemetry
        final float KP = 0.0035f;
        final float KI = 0.00009f;
        final float KD = 0.0005f;
        float I = 0.0f;
        long startTime = System.currentTimeMillis();
        float startError = 0;
        while((leftMotor.getCurrentPosition() <= distance || rightMotor.getCurrentPosition() <= distance) && opModeIsActive()){
            //should probably allow for some tolerance, maybe like 4 ticks
            int leftMotorPos=leftMotor.getCurrentPosition();
            int rightMotorPos=rightMotor.getCurrentPosition();
            long deltaTime = System.currentTimeMillis() - startTime;
            if(power<maxPower){
                power+=maxPower/rampTime*(deltaTime)/100;
            }
            float P = leftMotorPos - rightMotorPos;
            float D = (P - startError)/deltaTime;
            //Calculate integral with change in time from the previous iteration of the loop
            I = -(I + P*deltaTime);
            //Set limits to prevent integral windup
            //if(I>1) I=1;
            //else if(I<-1) I=-1;
            //Placed directly into the motor functions based on which way it is supposed to respond
            float correction = KP * P + KI * I * (power/maxPower) + KD * D;
            //Limit motor power from range of 0 to 100
            float leftcorrection = power-correction;
            if(leftcorrection>100) leftcorrection=100; if(leftcorrection<0) leftcorrection=0;
            float rightcorrection = power+correction;
            if(rightcorrection>100) rightcorrection=100; if(rightcorrection<0) rightcorrection=0;
            leftcorrection*=power/maxPower;
            rightcorrection*=power/maxPower;
            runDriveTrain(leftcorrection, rightcorrection);
            telemetry.addData("Correction Factor:", correction);
            telemetry.addData("left motor", leftMotor.getCurrentPosition());
            telemetry.addData("right motor", rightMotor.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.addData("right correction", rightcorrection);
            telemetry.addData("left correction", rightcorrection);
            telemetry.addData("P", P);
            telemetry.addData("I: ", I);
            telemetry.addData("D: ", D);
            telemetry.update();
            //Reset timer
            startTime = System.currentTimeMillis();
            startError = P;
            //Pause for one millisecond in order to not overwhelm the I loop.
            try{
                Thread.sleep(1);
            } catch(Exception e){

            }
        }
        runDriveTrain(0.0f, 0.0f);
        resetEncoders();
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void resetEncoders(){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runDriveTrain(float leftPower,float rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
    public void pressButton(){
        //once colors are detected line up and press the button
        //line up with button
        buttonPresser.setPosition(1);
        try{
            sleep(1000);
        } catch(Exception e){
            System.out.println(e.getMessage());
        }
        buttonPresser.setPosition(0);
        try{
            sleep(2200);
        } catch(Exception e){
            System.out.println(e.getMessage());
        }
        String color=null;
        if(colorSensor.blue()>4 && colorSensor.blue()>colorSensor.red()){
            color="blue";
        }
        else if(colorSensor.red()>4 && colorSensor.red()>colorSensor.blue()){
            color="red";
        }
        if(color!=MYCOLOR){
            try{
            sleep(3000);}
            catch(Exception e){
                System.out.println(e.getMessage());
            }
            buttonPresser.setPosition(1);
        }
        try{
            sleep(1000);}
        catch(Exception e){
            System.out.println(e.getMessage());
        }
        buttonPresser.setPosition(0);
        //end and move the f on
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        leftMotor=hardwareMap.dcMotor.get("left_drive");
        rightMotor=hardwareMap.dcMotor.get("right_drive");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        gyroSensor=hardwareMap.gyroSensor.get("gyro");
        colorSensor=hardwareMap.colorSensor.get("color");

        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            telemetry.addData("Calibrating", true);
            telemetry.update();
        }
        telemetry.addData("Between Phases", ": HMM");
        telemetry.update();
        travelMeters(1.524f, 0.3f);//this is triggering
        turnLeftDegrees(90);//this is not
        travelMeters(1.8288f);
        turnRightDegrees(90);
        travelMeters(.2f);//go to color beacon??????
        //pressButton();
        travelMeters(.2f);
        //pressButton();
        turnRightDegrees(140);//turn to point at center
        travelMeters(2);//go to the center and park

        while(opModeIsActive()){
            idle();
        }
    }
}
