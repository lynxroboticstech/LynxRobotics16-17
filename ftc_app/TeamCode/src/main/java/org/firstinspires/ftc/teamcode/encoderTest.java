package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import java.lang.*;
/**
 * Created by Student on 9/15/2016.
 */
@Autonomous(name="EncoderTest", group="Iterative Opmode")
public class encoderTest extends LinearOpMode {

    HardwareMap hw=null;
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    double gyroValueWhenStraight=.581622678;
    float ticks = 1120;//1440?
    float circumference = (float)(3.81 * 3.81 * Math.PI); //1.5 inches -> cm; also, to whoever wrote this, this actually gives the area
    boolean encoderFlag = true;
    public void encoderDrive(float distance, float power){
        resetEncoders();
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //p(id) loop, telemetry
        float KP = 0.005f;
        float KI = 0.004f;
        float I = 0.0f;
        long startTime = System.currentTimeMillis();
        while((leftMotor.getCurrentPosition() <= distance || rightMotor.getCurrentPosition() <= distance) && opModeIsActive()){
            //should probably allow for some tolerance, maybe like 4 ticks
            int leftMotorPos=leftMotor.getCurrentPosition();
            int rightMotorPos=rightMotor.getCurrentPosition();
            long deltaTime = System.currentTimeMillis() - startTime;
            float P = leftMotorPos - rightMotorPos;
            //Calculate integral with change in time from the previous iteration of the loop
            I = -(I + P*deltaTime);
            //Set limits to prevent integral windup
            if(I>100/KI) I=100/KI;
            else if(I<-100/KI) I=-100/KI;
            //Placed directly into the motor functions based on which way it is supposed to respond
            float correction = KP * P + KI * I;
            //Limit motor power from range of 0 to 100
            float leftcorrection = power-correction;
            if(leftcorrection>100) leftcorrection=100; if(leftcorrection<0) leftcorrection=0;
            float rightcorrection = power+correction;
            if(rightcorrection>100) rightcorrection=100; if(rightcorrection<0) rightcorrection=0;
            runDriveTrain(leftcorrection, rightcorrection);
            telemetry.addData("Correction Factor: ", correction);
            telemetry.addData("left motor", leftMotor.getCurrentPosition());
            telemetry.addData("right motor", rightMotor.getCurrentPosition());
            telemetry.addData("KP", P);
            telemetry.addData("I: ", I);
            telemetry.update();
            //Reset timer
            startTime = System.currentTimeMillis();
            //Pause for one millisecond in order to not overwhelm the I loop.
            try{
                Thread.sleep(1);
            } catch(Exception e){

            }
        }
        runDriveTrain(0.0f, 0.0f);
        resetEncoders();
    }
    public void encoderDriveMeters(float distance, float power) {
        encoderDrive(distance * ticks / circumference, power);
    }
    public void travelMeters(float d) {
        travelMeters(d, 0.3f);
    }
    public void travelMeters(float d, float power) {
        d=d*100;
        //convert d to cm
        float distancePerTick=10000/218;//ticks/cm -- experimentally determined
        if(d > 0) {
            encoderDrive(d*distancePerTick, power);
        } else {
            encoderDrive(-d*distancePerTick, power);
        }
    }
    /*
    public void turnXDegreesBasic(double X) {
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        runDriveTrain(-.3f, .3f);
        double gyroPerDegree=.84f;//Wrong,
        while (zCounter - (gyroPerDegree*X)<0){
            runDriveTrain(-.3f, .3f);
            zCounter += ((double)getGyroRotation(gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis();
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("distance requested",gyroPerDegree*X);
            telemetry.addData("gyro value",getGyroRotation(gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }
        runDriveTrain(0,0);
    }

    public void turnXDegreesuUsingHardCodedValues(double X) {
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        runDriveTrain(-.3f, .3f);
        double gyroPerDegree=.84f;//Wrong,
            if(X==90){
                gyroPerDegree=.84;
            }
        if(X==180){
            gyroPerDegree=.9;
        }
        if(X==720){
            gyroPerDegree=.95;
        }
        if(X==45){
            gyroPerDegree=.73;
        }
        if(X==30){
            gyroPerDegree=.67;
        }
        while (zCounter - (gyroPerDegree*X)<0){
            runDriveTrain(-.3f, .3f);
            zCounter += ((double)getGyroRotation(gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis();
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("distance requested",gyroPerDegree*X);
            telemetry.addData("gyro value",getGyroRotation(gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }
        runDriveTrain(0,0);
    }
    public void turnXDegrees(double X,double gyroPerDegree) {
        //overload function
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        runDriveTrain(-.3f, .3f);
        while (zCounter - (gyroPerDegree*X)<0) {
            runDriveTrain(-.3f, .3f);
            zCounter += ((double) getGyroRotation(gyroSensor) - gyroValueWhenStraight) * (double) (System.currentTimeMillis() - previousT);
            previousT = System.currentTimeMillis();
        }
        runDriveTrain(0,0);
    }
    public double encoderToAngle(double enc) { //converts encoder ticks to angle in degrees
        double wheelDiam = 33.8; //cm between wheels
        float cmPerTick=218/10000; //cm per encoder tick
        double factor = cmPerTick * 360 / Math.PI / wheelDiam; //Mathemtically determined, Theta = factor * encoder ticks
        return enc * factor; //multiply current encoder ticks by factor, gives theta in degrees
    }
    public double angleToEncoder(double angle) { //converts encoder ticks to angle in degrees
        double wheelDiam = 33.8; //cm between wheels
        float cmPerTick=218/10000; //cm per encoder tick
        double factor = cmPerTick * 360 / Math.PI / wheelDiam; //Mathemtically determined, Theta = factor * encoder ticks
        return angle / factor; //multiply current encoder ticks by factor, gives theta in degrees
    }

    public void turnByEncoders(boolean direction, int degrees) { //directoin = true for left is the plan later, not implemented yet
        if(direction) {
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        float power = .3f;
        float distance = (float) angleToEncoder(degrees);
        resetEncoders();
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //p(id) loop, telemetry
        final float KP = 0.005f;
        while(leftMotor.getCurrentPosition() <= distance || rightMotor.getCurrentPosition() <= distance){

            if(leftMotor.getCurrentPosition() > rightMotor.getCurrentPosition()){
                float correction = KP * (leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition());
                runDriveTrain((power - correction), (power + correction));

            }else if(leftMotor.getCurrentPosition() < rightMotor.getCurrentPosition()){
                float correction = KP * (rightMotor.getCurrentPosition() - leftMotor.getCurrentPosition());
                runDriveTrain((power + correction), (power - correction));

            }else{
                runDriveTrain(power, power);
            }

            telemetry.addData("degrees traveled", encoderToAngle(leftMotor.getCurrentPosition()));
            telemetry.update();
        }
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("Encoder has stopped.");
        telemetry.update();
        runDriveTrain(0.0f, 0.0f);
        resetEncoders();
    }
    */
    public void runOpMode() throws InterruptedException{
        //hf=new HardwareFunctions(hardwareMap);
        waitForStart();
        //travelMeters(1); //218/10000
        //turnXDegrees(90);
        //calibrateEncoder(1, 0.4f);
       /* for (int i = 0; i < 12*4; i++) {
            turnXDegrees(15,.6f);
            sleep(500);
        }*/
        leftMotor=hardwareMap.dcMotor.get("left_drive");
        rightMotor=hardwareMap.dcMotor.get("right_drive");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        travelMeters(1);
        while(opModeIsActive()) {
            idle();
        }

    }

    private void resetEncoders(){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runDriveTrain(float leftPower,float rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
    /*
    public void calibrateEncoder(float dist, float power) {
        travelMeters(dist, power, false);
        if(leftMotor.getCurrentPosition() > rightMotor.getCurrentPosition()) {

        } else if (leftMotor.getCurrentPosition() < leftMotor.getCurrentPosition()) {

        } else {
            //CORRECTLY CALIBRATED
        }
    } */
}
