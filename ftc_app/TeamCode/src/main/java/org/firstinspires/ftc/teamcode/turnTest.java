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

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Random;

/**
 * Created by Student on 10/18/2016.
 */
@Autonomous(name="Turn test", group="Iterative Opmode")
public class turnTest extends LinearOpMode {
    public GyroSensor gyroSensor=null;
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor   = null;

    public void turnRightDegrees(double X) {
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            telemetry.addData("Calibrating", true);
            telemetry.update();
        }
        while (gyroSensor.getHeading() < X - 18||gyroSensor.getHeading()>357) {
            leftMotor.setPower(-.2);
            rightMotor.setPower(.2);
            telemetry.addData("current angle: ", gyroSensor.getHeading());
            telemetry.addData("Fast", true);
            telemetry.update();
        }
        while (gyroSensor.getHeading() <= X) {
            leftMotor.setPower(-.07);
            rightMotor.setPower(.07);
            telemetry.addData("current angle: ", gyroSensor.getHeading());
            telemetry.addData("Slow", true);
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void turnLeftDegrees(double X){
            //turn left
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            telemetry.addData("Calibrating", true);
            telemetry.update();
        }
            X=360-X;
            while(gyroSensor.getHeading()>X+18||gyroSensor.getHeading()<3){
                leftMotor.setPower(.2);
                rightMotor.setPower(-.2);
                telemetry.addData("current angle: ",gyroSensor.getHeading());
                telemetry.addData("Fast",true);
                telemetry.update();
            }
            while(gyroSensor.getHeading()>=X){
                leftMotor.setPower(.07);
                rightMotor.setPower(-.07);
                telemetry.addData("current angle: ",gyroSensor.getHeading());
                telemetry.addData("Slow",true);
                telemetry.update();
            }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor=hardwareMap.dcMotor.get("left_drive");
        rightMotor=hardwareMap.dcMotor.get("right_drive");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        gyroSensor=hardwareMap.gyroSensor.get("gyro");
        turnRightDegrees(135);
        telemetry.addData("current angle: ",gyroSensor.getHeading());
        telemetry.addData("Calibrating",false);
        telemetry.update();
        while(opModeIsActive()){
            idle();
        }
    }
}
