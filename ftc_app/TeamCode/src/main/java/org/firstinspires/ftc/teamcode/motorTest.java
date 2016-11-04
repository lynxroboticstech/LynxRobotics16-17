package org.firstinspires.ftc.teamcode;

import android.net.LinkAddress;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Student on 9/10/2016.
 */
@Autonomous(name="StupidKagoo Test", group="Iterative Opmode")
public class motorTest extends LinearOpMode {
    public DcMotor leftMotor   = null;
    HardwareMap hwMap           =  null;
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = hardwareMap;
        leftMotor = hwMap.dcMotor.get("motor");
        leftMotor.setPower(0);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
            leftMotor.setPower(1);
            sleep(120);
            leftMotor.setPower(0);
            sleep(10);
            leftMotor.setPower(-1);
            sleep(120);
            leftMotor.setPower(0);
            idle();

    }
}

