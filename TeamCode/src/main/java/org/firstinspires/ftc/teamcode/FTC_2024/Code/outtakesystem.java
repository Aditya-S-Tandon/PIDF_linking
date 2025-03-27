package org.firstinspires.ftc.teamcode.FTC_2024;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robotconstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class outtakesystem {

    DcMotor IntakeMotor = null;
    Servo IntakeServo = null;

    public outtakesystem(HardwareMap hardwareMap)  {
        IntakeServo = hardwareMap.get(Servo.class, "IS");
        IntakeMotor = hardwareMap.dcMotor.get("IM");
    }
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
        }
    }
    public void grab(int state)  {

        //IntakeServo.setPosition(Robotconstants.servo_bucket_output);
        //safeWaitSeconds(0.5);
        //IntakeMotor.setPower(-Robotconstants.intake_motor_power);
        //(0.25);
        if(state==1) {
            //safeWaitSeconds(0.4);
            IntakeServo.setPosition(Robotconstants.servo_bucket_intake);
            safeWaitSeconds(0.4);
            IntakeMotor.setPower(Robotconstants.outtake_motor_power);
            //safeWaitSeconds(0.1);
        }
        else {
            IntakeMotor.setPower(0.);
        }
    }
}

