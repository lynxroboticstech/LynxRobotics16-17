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
    float ticks = 1120;//1440?
    float circumference = (float)(3.81 * 3.81 * Math.PI); //1.5 inches -> cm; also, to whoever wrote this, this actually gives the area
    boolean encoderFlag = true;
    public void encoderDrive(float distance, float power){
            hf.resetEncoders();
            hf.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hf.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //p(id) loop, telemetry
            final float KP = 0.005f;
            final float leftCalibration=1000f;
            final float rightCalibration=0f;
            final float calibration = 0;
        while((hf.leftMotor.getCurrentPosition() <= distance || hf.rightMotor.getCurrentPosition() <= distance) && opModeIsActive()){
            //should probably allow for some tolerance, maybe like 4 ticks
            if(hf.leftMotor.getCurrentPosition() > hf.rightMotor.getCurrentPosition()){
                float correction = KP * (hf.leftMotor.getCurrentPosition() + calibration - hf.rightMotor.getCurrentPosition());
                hf.runDriveTrain((power - correction)+leftCalibration, (power + correction)+rightCalibration);
                telemetry.addData("Correction Facrot Right", correction);
            }else if(hf.leftMotor.getCurrentPosition() < hf.rightMotor.getCurrentPosition()){
                float correction = KP * (hf.rightMotor.getCurrentPosition() + calibration - hf.leftMotor.getCurrentPosition());
                hf.runDriveTrain((power + correction)+leftCalibration, (power - correction)+rightCalibration);
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

    public void turnXDegrees(double X) {
        hf.gyroSensor.calibrate();
        double previousT=System.currentTimeMillis();
        while(hf.gyroSensor.isCalibrating()){}
        double gyroValueWhenStraight=.581622678;//experimentally determined, probably
        double zCounter = 0;
        hf.runDriveTrain(-.5f, 0);
        double gyroPerDegree=1765/90;
        while (zCounter < gyroPerDegree*X){
            hf.runDriveTrain(-.5f, 0);
            zCounter += (hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight)*previousT;
            previousT=System.currentTimeMillis()-previousT;
            telemetry.addData("zCounter",zCounter);
            telemetry.addData("gyro value",hf.getGyroRotation(hf.gyroSensor) - gyroValueWhenStraight);
            telemetry.update();
        }
    }
    public double encoderToAngle(int enc) { //converts encoder ticks to angle in degrees
        double wheelDiam = 33.8; //cm between wheels
        float cmPerTick=218/10000; //cm per encoder tick
        double factor = cmPerTick * 360 / Math.PI / wheelDiam; //Mathemtically determined, Theta = factor * encoder ticks
        return enc * factor; //multiply current encoder ticks by factor, gives theta in degrees
    }
    public double angleToEncoder(int angle) { //converts encoder ticks to angle in degrees
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
      //  waitForStart();
        //travelMeters(1);
        // travelMeters(1); //218/10000
        turnXDegrees(90);
        //calibrateEncoder(1, 0.4f);

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
