package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/20/2016.
 */
@TeleOp(name="joystickTest")
public class joystickTest extends OpMode {
    HardwareMap hw = null;
    HardwareFunctions hf = null;

    public void init(){
        hf = new HardwareFunctions(hardwareMap);

        hf.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hf.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(){
        hf.leftMotor.setPower(gamepad1.left_stick_y);
        hf.rightMotor.setPower(gamepad1.right_stick_y);
    }
}
