package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.net.LinkAddress;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Student on 9/10/2016.
 */

public class HardwareFunctions{
    HardwareMap hwMap           =  null;

    public DcMotor leftMotor   = null;
    public DcMotor rightMotor   = null;
    public ColorSensor colorSensor = null;


    public HardwareFunctions(HardwareMap hardwareMap){
        hwMap = hardwareMap;
        leftMotor = hwMap.dcMotor.get("left_drive");
        leftMotor.setPower(0);
        rightMotor = hwMap.dcMotor.get("right_drive");
        rightMotor.setPower(0);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        colorSensor = hardwareMap.colorSensor.get("color sensor");
        colorSensor.enableLed(false);
    }

    public void runDriveTrain(float rightPower,float leftPower){
        leftMotor.setPower(rightPower);
        rightMotor.setPower(leftPower);
    }


    //TODO:FILL IN FUNCTIONS
    public void goStraightXDistance(float X){

    }
    public void turnXDegrees(float X){

    }
    public float getTouchSensorData(TouchSensor touchSensor){
        return 1;
    }

    public float getColorSensorBlue(){
        return this.colorSensor.blue();
    }
    public float getColorSensorRed(){
        return this.colorSensor.red();
    }
    public float getColorSensorGreen(){
        return this.colorSensor.green();
    }
    public void setColorSensorLight(boolean b) {
        colorSensor.enableLed(b);
    }
    /*public static class ColorAlgorithm {
        public boolean isSensorInRange() {
            return false;
        }
        public BeaconColor getBeaconColor() { //0 for
            return null;
        }
        public int getRange() { //0 for very close, 1 for small range, 2 for far ranges
            return 0;
        }
        public boolean inRange() { //return false for range
            return false;
        }
    }
    public enum BeaconColor {
        RED,
        BLUE;
    }
    public static class Navigation {

    }*/

    public float getOpticalDistanceSensorData(OpticalDistanceSensor ODS) {
        return 1;
    }


}

