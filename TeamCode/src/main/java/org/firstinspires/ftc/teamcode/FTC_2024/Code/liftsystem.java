package org.firstinspires.ftc.teamcode.FTC_2024;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.config.Robotconstants;
import org.firstinspires.ftc.teamcode.config.subsystem.outtakesystem;


/** This is a subsystem, for the claw of our robot
 * Here we make methods to manipulate the servos
 * We also import RobotConstants to get the positions of the servos.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

public class liftsystem {

    public DcMotor LiftMotor;
    public Servo IntakeServo;
    public Servo BigWrist;
   // public outtakesystem outtake;
    /** This is the constructor for the subsystem, it maps the servos to the hardwareMap.
     * The device names should align with the configuration names on the driver hub.
     * To use this subsystem, we have to import this file, declare the subsystem (private ClawSubsystem claw;),
     * and then call the below constructor in the init() method. */

    public liftsystem(HardwareMap hardwareMap) {
        BigWrist = hardwareMap.get(Servo.class, "BW");
        IntakeServo = hardwareMap.get(Servo.class, "IS");
        LiftMotor = hardwareMap.get(DcMotor.class,"LM");

    }
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
        }
    }
public void elevate(int state, outtakesystem outtake) {
if (state==1){
            safeWaitSeconds(1.);
        }


    ElapsedTime runtime = new ElapsedTime();

    LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //set encoders to 0
    LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Zero Power Behavior
    LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    String LiftMotorCurrentDirection = "up";

    //Lift
    runtime.reset();
    LiftMotor.setPower(-Robotconstants.lift_motor_power);
    LiftMotor.setTargetPosition(Robotconstants.extention_target);
    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LiftMotorCurrentDirection = "up";
    //Remove  Power from the lift Motor
    while (LiftMotorCurrentDirection.equals("up") && (LiftMotor.getCurrentPosition() < Robotconstants.extention_target) && (runtime.seconds() < 3)) {
        //extention_power = 0;
        //ExtentionMotor.setPower(extenstion_power);
        //ExtentionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        if (LiftMotor.getCurrentPosition() > 790) {
            BigWrist.setPosition(Robotconstants.big_wrist_position);
        }
        //telemetry.addData("Lift motor position", LiftMotor.getCurrentPosition());
        //telemetry.update();
    }
    // idle();

    safeWaitSeconds(1.5);
    BigWrist.setPosition(Robotconstants.servoposition);
    safeWaitSeconds(0.75);
    //IntakeServo.setPosition(Robotconstants.servo_bucket_output);
    LiftMotor.setPower(-Robotconstants.lift_motor_power);
    LiftMotor.setTargetPosition(Robotconstants.retraction_target);
    LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    LiftMotorCurrentDirection = "down";

    while (LiftMotorCurrentDirection.equals("down") && (LiftMotor.getCurrentPosition() > Robotconstants.retraction_target)) {

        //telemetry.addData("Lift motor position", LiftMotor.getCurrentPosition());
        //telemetry.update();
        outtake.grab(2);
    }
//idle();
    //Remove  Power from the lift Motor
    //Robotconstants.lift_motor_power = 0;
    LiftMotor.setPower(0.);
    LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
