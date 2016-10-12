package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/15/2016.
 */
@Autonomous(name="EncoderTest", group="Iterative Opmode")
public class EncoderTest extends LinearOpMode {
    HardwareMap hw=null;
    HardwareFunctions hf = null;
    double gyroValueWhenStraight=.581622678;
    float ticks = 1120;//1440?
    float circumference = (float)(3.81 * 3.81 * Math.PI); //1.5 inches -> cm; also, to whoever wrote this, this actually gives the area
    boolean encoderFlag = true;
    public void encoderDrive(float distance, float power){
        hf.resetEncoders();
        hf.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hf.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //p(id) loop, telemetry
        final float KP = 0.005f;
        while((hf.leftMotor.getCurrentPosition() <= distance || hf.rightMotor.getCurrentPosition() <= distance) && opModeIsActive()){
            //should probably allow for some tolerance, maybe like 4 ticks
            int leftMotorPos=hf.leftMotor.getCurrentPosition();
            int rightMotorPos=hf.rightMotor.getCurrentPosition()+250;
            if( leftMotorPos>rightMotorPos ){
                float correction = KP * (leftMotorPos - rightMotorPos);
                hf.runDriveTrain((power - correction), (power + correction));
                telemetry.addData("Correction Facrot Right", correction);
            }else if(hf.leftMotor.getCurrentPosition() <rightMotorPos){
                float correction = KP * (rightMotorPos - leftMotorPos);
                hf.runDriveTrain((power + correction), (power - correction));
                telemetry.addData("Correction Facrot Left", correction);

            }else{
                hf.runDriveTrain(power, power);
            }

            telemetry.addData("left motor", hf.leftMotor.getCurrentPosition());
            telemetry.addData("right motor", hf.rightMotor.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addLine("Encoder has stopped.");
        telemetry.update();
        hf.runDriveTrain(0.0f, 0.0f);
        hf.resetEncoders();
    }
    public void encoderDriveMeters(float distance, float power) {
        encoderDrive(distance * ticks / circumference, power);
    }
    public void travelMeters(float d) {
        travelMeters(d, 1.0f);
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

    public void turnXDegreesBasic(double X) {
        hf.gyroSensor.calibrate();
        while(hf.gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        hf.runDriveTrain(-.3f, .3f);
        double gyroPerDegree=.84f;//Wrong,
        while (zCounter - (gyroPerDegree*X)<0){
            hf.runDriveTrain(-.3f, .3f);
            zCounter += ((double)hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis();
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("distance requested",gyroPerDegree*X);
            telemetry.addData("gyro value",hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }
        hf.runDriveTrain(0,0);
    }
    public void turnXDegreesuUsingHardCodedValues(double X) {
        hf.gyroSensor.calibrate();
        while(hf.gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
        hf.runDriveTrain(-.3f, .3f);
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
            hf.runDriveTrain(-.3f, .3f);
            zCounter += ((double)hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis();
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("distance requested",gyroPerDegree*X);
            telemetry.addData("gyro value",hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }
        hf.runDriveTrain(0,0);
    }
    public void turnXDegrees(double X,double gyroPerDegree) {
        //overload function
        hf.gyroSensor.calibrate();
        while(hf.gyroSensor.isCalibrating()){}
        double previousT=System.currentTimeMillis();
        //experimentally determined, probably.  Probably wrong.  Fix
        double zCounter = 0;
<<<<<<< HEAD
        hf.runDriveTrain(-.3f, .3f);
        while (zCounter - (gyroPerDegree*X)<0){
            hf.runDriveTrain(-.3f, .3f);
            zCounter += ((double)hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight)*(double)(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis();
=======
        hf.runDriveTrain(-.5f, 0);
        double gyroPerDegree=1765/90;
        while (zCounter < gyroPerDegree*X){
            hf.runDriveTrain(-.5f, 0);
            zCounter += (hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight)*(System.currentTimeMillis()-previousT);
            previousT=System.currentTimeMillis()-previousT;
>>>>>>> origin/master
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("distance requested",gyroPerDegree*X);
            telemetry.addData("gyro value",hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }
        hf.runDriveTrain(0,0);
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
            hf.rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            hf.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            hf.rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hf.leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        float power = .3f;
        float distance = (float) angleToEncoder(degrees);
        hf.resetEncoders();
        hf.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hf.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //p(id) loop, telemetry
        final float KP = 0.005f;
        while(hf.leftMotor.getCurrentPosition() <= distance || hf.rightMotor.getCurrentPosition() <= distance){

            if(hf.leftMotor.getCurrentPosition() > hf.rightMotor.getCurrentPosition()){
                float correction = KP * (hf.leftMotor.getCurrentPosition() - hf.rightMotor.getCurrentPosition());
                hf.runDriveTrain((power - correction), (power + correction));

            }else if(hf.leftMotor.getCurrentPosition() < hf.rightMotor.getCurrentPosition()){
                float correction = KP * (hf.rightMotor.getCurrentPosition() - hf.leftMotor.getCurrentPosition());
                hf.runDriveTrain((power + correction), (power - correction));

            }else{
                hf.runDriveTrain(power, power);
            }

            telemetry.addData("degrees traveled", encoderToAngle(hf.leftMotor.getCurrentPosition()));
            telemetry.update();
        }
        hf.rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hf.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("Encoder has stopped.");
        telemetry.update();
        hf.runDriveTrain(0.0f, 0.0f);
        hf.resetEncoders();
    }

    public void runOpMode() throws InterruptedException{
        hf=new HardwareFunctions(hardwareMap);
        waitForStart();
        //travelMeters(1); //218/10000
        //turnXDegrees(90);
        //calibrateEncoder(1, 0.4f);
       /* for (int i = 0; i < 12*4; i++) {
            turnXDegrees(15,.6f);
            sleep(500);
        }*/
        encoderDriveMeters(1f,.4f);
        while(opModeIsActive()) {
            idle();
        }

    }
    /*
    public void calibrateEncoder(float dist, float power) {
        travelMeters(dist, power, false);
        if(hf.leftMotor.getCurrentPosition() > hf.rightMotor.getCurrentPosition()) {

        } else if (hf.leftMotor.getCurrentPosition() < hf.leftMotor.getCurrentPosition()) {

        } else {
            //CORRECTLY CALIBRATED
        }
    } */
}
