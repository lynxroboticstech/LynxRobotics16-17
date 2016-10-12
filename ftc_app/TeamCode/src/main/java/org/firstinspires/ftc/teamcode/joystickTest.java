package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/20/2016.
 */
@TeleOp(name="joystickTest")
public class JoystickTest extends OpMode {
    HardwareMap hwMap = null;
   // HardwareFunctions hf = null;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor forkliftMotor;



    public void init(){
        //hf = new HardwareFunctions(hardwareMap);
        hwMap = hardwareMap;
        leftMotor = hwMap.dcMotor.get("left_drive");
        leftMotor.setPower(0);
        rightMotor = hwMap.dcMotor.get("right_drive");
        forkliftMotor = hwMap.dcMotor.get("forklift");
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop(){
        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);
        forkliftMotor.setPower(gamepad2.left_stick_y);
    }
}
