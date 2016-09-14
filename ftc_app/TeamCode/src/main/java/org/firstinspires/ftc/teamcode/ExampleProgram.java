package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.testcode.TestTelemetry;
import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.robotcore.internal.TelemetryInternal;
import org.firstinspires.ftc.teamcode.HardwareFunctions;
/**
 * Created by Student on 9/13/2016.
 */
@Autonomous(name="ExampleProgram", group="Iterative Opmode")
public class ExampleProgram extends LinearOpMode {
HardwareMap hw=null;
    //comment
   public void runOpMode() throws InterruptedException{
        HardwareFunctions hf=new HardwareFunctions(hardwareMap);
        waitForStart();
       int i = 0;
       while(opModeIsActive()){
           telemetry.addData("", hf.getColorSensorBlue());
           telemetry.update();
           /*if(hf.getColorSensorBlue()>.2){
           hf.runDriveTrain(20,20);}
           else if(hf.getColorSensorRed()>.2){
               hf.runDriveTrain(-20,-20);
           } else {
               hf.runDriveTrain(0,0);
           }*/
           //hf.runDriveTrain(hf.getColorSensorBlue(), hf.getColorSensorGreen());
       }
    }


}
