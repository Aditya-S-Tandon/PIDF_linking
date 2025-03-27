package org.firstinspires.ftc.teamcode.FTC_2024;

// Import the necessary FTC SDK classes
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robotconstants;

@Autonomous(name = "BreakBeamTesting")
public class BreakBeamTesting extends LinearOpMode {

    // Declare the DigitalInput object for the break beam sensor
    private DigitalChannel beamSensor;
    Servo IntakeServo = null;
    DcMotor IntakeMotor = null;
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
        }
    }
    @Override
    public void runOpMode() {

        // Initialize the hardware
        beamSensor = hardwareMap.get(DigitalChannel.class, "beamSensor");
        IntakeServo = hardwareMap.get(Servo.class, "IS");
        IntakeMotor = hardwareMap.dcMotor.get("IM");

        // Set the sensor mode to input
        beamSensor.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Main loop that runs while the opmode is active
        while (opModeIsActive()) {
            beamSensor.setState(true);
            IntakeServo.setPosition(Robotconstants.servo_bucket_output);
            //safeWaitSeconds(0.5);
            IntakeMotor.setPower(-Robotconstants.intake_motor_power);
            //(0.25);
            //IntakeServo.setPosition(Robotconstants.servo_bucket_intake);
            safeWaitSeconds(0.6);
            IntakeServo.setPosition(Robotconstants.servo_bucket_intake);
            safeWaitSeconds(0.4);
            IntakeMotor.setPower(Robotconstants.outtake_motor_power);
            
            // Check the beam sensor state and output the result
            if (beamSensor.getState()) {
                // Beam is NOT broken, sensor is reading HIGH
                telemetry.addData("Beam Status", "Not Broken");
            } else {
                // Beam is broken, sensor is reading LOW
                telemetry.addData("Beam Status", "  Broken");
            }

            // Update the telemetry
            telemetry.update();
        }
    }
}
