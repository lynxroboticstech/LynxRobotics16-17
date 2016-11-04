package org.firstinspires.ftc.teamcode;

import android.net.LinkAddress;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Student on 9/10/2016.
 */
@Autonomous(name="TESTING PROGRAM", group="Iterative Opmode")
public class WorkingTestFile extends LinearOpMode {
    //THIS IS PURELY FOR TESTING FUNCTIONS AT A TIME
    //DO NOT EXTEND HF.
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor   = null;
    HardwareMap hwMap           =  null;
    void turnByDegreesUsingTime(double degrees){
        double millisecondsPerDegree=1;
        double startingTime=(double)(System.currentTimeMillis());
        int x=0;
        while(System.currentTimeMillis()-startingTime<millisecondsPerDegree*degrees){
            leftMotor.setPower(-.3);
            rightMotor.setPower(.3);
            x++;
            telemetry.addData("x: ",x);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = hardwareMap;
        leftMotor = hwMap.dcMotor.get("left_drive");
        leftMotor.setPower(0);
        rightMotor = hwMap.dcMotor.get("right_drive");
        rightMotor.setPower(0);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        turnByDegreesUsingTime(10000);
        while (opModeIsActive()) {
        }
    }
}

