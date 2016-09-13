package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Student on 9/12/2016.
 */
public class colorSensorTest {

    @Autonomous(name = "Color Sensor Test", group = "Iterative Opmode")
    public class SensorMRColor extends LinearOpMode {
        public DcMotor leftMotor   = null;
        public DcMotor rightMotor   = null;
        ColorSensor colorSensor;    // Hardware Device Object
        HardwareMap hwMap           =  null;


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



                // hsvValues is an array that will hold the hue, saturation, and value information.
                float hsvValues[] = {0F, 0F, 0F};

                // values is a reference to the hsvValues array.
                final float values[] = hsvValues;

                // get a reference to the RelativeLayout so we can change the background
                // color of the Robot Controller app to match the hue detected by the RGB sensor.
                final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

                // bPrevState and bCurrState represent the previous and current state of the button.
                boolean bPrevState = false;
                boolean bCurrState = false;

                // bLedOn represents the state of the LED.
                boolean bLedOn = true;

                // get a reference to our ColorSensor object.
                colorSensor = hardwareMap.colorSensor.get("color sensor");

                // Set the LED in the beginning
                colorSensor.enableLed(bLedOn);

                // wait for the start button to be pressed.
                waitForStart();

                // while the op mode is active, loop and read the RGB data.
                // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
                while (opModeIsActive()) {


                    // send the info back to driver station using telemetry function.
                    if (colorSensor.red() > 10) {
                        rightMotor.setPower(20);
                    }
                    if (colorSensor.blue() > 10) {
                        leftMotor.setPower(20);
                    }
                }

            }
        }
    }




