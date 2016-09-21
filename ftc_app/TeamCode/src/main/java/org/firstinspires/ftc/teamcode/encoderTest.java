package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/15/2016.
 */
@Autonomous(name="EncoderTest", group="Iterative Opmode")
public class encoderTest extends LinearOpMode {
    HardwareMap hw=null;
    HardwareFunctions hf = null;
    float ticks = 1120;
    float circumference = (float)(3.81 * 3.81 * Math.PI); //centimers, exacty 1.5 inches
    public void encoderDrive(float distance, float power){
        hf.resetEncoders();
        hf.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hf.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hf.runDriveTrain(power, power);
        while(hf.leftMotor.getCurrentPosition() <= distance || hf.rightMotor.getCurrentPosition() <= distance){
            telemetry.addData("left motor", hf.leftMotor.getCurrentPosition());
            telemetry.addData("right motor", hf.rightMotor.getCurrentPosition());
        }
        telemetry.addData("encoder status", "done");
        hf.runDriveTrain(0.0f, 0.0f);
        hf.resetEncoders();
    }
    public void encoderDriveMeters(float distance, float power) {
        encoderDrive(distance * ticks / circumference, power);
    }
    public void travelMeters(float d) { //NOT DONE
        float power = 1.0f;
        if(d > 0) {
            encoderDriveMeters(d, power);
        } else {
            encoderDriveMeters(-d, power);
        }
    }

    public void runOpMode() throws InterruptedException{
        hf=new HardwareFunctions(hardwareMap);
        waitForStart();
        encoderDrive(1120, 1.0f);
    }
}
