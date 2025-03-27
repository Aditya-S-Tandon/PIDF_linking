package org.firstinspires.ftc.teamcode.FTC_2024;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="figuring_out_pid")
public class figuring_out_pid extends LinearOpMode {

    DcMotorEx LiftMotor;

    double integralSum = 0;
    double Kp = 0; //tune Kp Ki and Kd Sri!!!
    double Ki = 0; // to tune the contoler, use the tecniques from control ftc, you want to tune the proportional first until it meets the correct setting
    double Kd = 0;
    double Kf = 0; //  the feedforward component and does not rely on measurments
// if you start by setting everthing to 0 and then just increasing kf untill you have roughly met the velocity, and then tuning Kp Ki and Kd the velocity PID control will be better and more reliable

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        LiftMotor = hardwareMap.get(DcMotorEx.class, "motor");
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //makes it so that when you are doing the setpower, the motor is not usint the feedback, if ran with encoder, it will run at a different speed, but the motor will only run at 80% like this

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double power = PIDControl(100, LiftMotor.getVelocity()); // needs to be tuned by Sri, and assuming that the Kp Ki and Kd values are tuned correctly, the motor will spin at 100 ticks per second
            LiftMotor.setPower(power);
        }
    }


    public double PIDControl(double reference, double state) {
        double error = reference - state;
        double dt = timer.seconds();
        timer.reset(); // reset after the derivative is found

        integralSum += error * dt;
        integralSum = Math.max(Math.min(integralSum, 1.0), -1.0); // stops the motor from winding up

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (error * Kp) + (derivative * Kd) + (integralSum *Ki) + (reference * Kf);
        return output;
    }
}