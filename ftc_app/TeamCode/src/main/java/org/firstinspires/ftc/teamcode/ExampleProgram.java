package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
       while(opModeIsActive()){
           if(hf.getColorSensorBlue(hf.colorSensor)>0){
           hf.runDriveTrain(20,20);}
           else if(hf.getColorSensorRed(hf.colorSensor)>0){
               hf.runDriveTrain(-20,-20);
           }
       }
    }


}
