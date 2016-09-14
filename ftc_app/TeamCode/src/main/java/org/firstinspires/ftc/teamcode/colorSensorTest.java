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
@Autonomous(name="ColorSensorTest", group="Iterative Opmode")
public class ColorSensorTest extends LinearOpMode {
HardwareMap hw=null;
    //comment
   public void runOpMode() throws InterruptedException{
        HardwareFunctions hf=new HardwareFunctions(hardwareMap);
        waitForStart();
       int i = 0;
       hf.setColorSensorLight(false);
       while(opModeIsActive()){
           //telemetry.addData("", hf.getColorSensorBlue() + " " + hf.getColorSensorGreen() + " " + hf.getColorSensorRed());
           if(hf.getColorSensorBlue() > 4 && hf.getColorSensorBlue() > hf.getColorSensorRed()) {
               telemetry.addData("", "(Blue)" + " Blue: " + hf.getColorSensorBlue() + " Red: " + hf.getColorSensorRed());
           } else if (hf.getColorSensorRed() > 4 && hf.getColorSensorBlue() < hf.getColorSensorRed()) {
               telemetry.addData("", "(Red)" + " Blue: " + hf.getColorSensorBlue() + " Red: " + hf.getColorSensorRed());
           } else {
               telemetry.addData("", "No Color" + " Blue: " + hf.getColorSensorBlue() + " Red: " + hf.getColorSensorRed());
           }
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
