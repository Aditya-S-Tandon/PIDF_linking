package org.firstinspires.ftc.teamcode.FTC_2024.Code;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import edu.wpi.first.wpilibj.controller.PIDController;


//@Config
@TeleOp(name="teleoppidf_robot")
public class PIDF_testing extends LinearOpMode {
    private PIDController controller;
    AnalogInput arm_encoder;

    public static double kp = 0, ki = 0, kd = 0;
    public static double kf = 0;
    public static int target = 0;
    int target1 = 100;
    int target2 = 230;
    private DcMotor HangMotor = null;

    double currentPosition;
    ;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            controller = new PIDController(kp, ki, kd);
            //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            HangMotor = hardwareMap.dcMotor.get("HM");
            //arm_encoder = hardwareMap.get(AnalogInput.class, "arm_encoder");

            HangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            HangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //Zero Power Behavior
            HangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // HangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            //  HangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // HangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad1.b) {
                kp = 0.0007;
                ki = 0;
                kd = 0;
                kf = 0.1;
                controller = new PIDController(kp, ki, kd);
                controller.setPID(kp, ki, kd);
                ElapsedTime runtime = new ElapsedTime();
                //double voltage = arm_encoder.getVoltage();
                //currentPosition = (voltage - 0) / 3.3 * 360;
                double pos_prev = 0.;
                //while (Math.abs(target - arm_encoder.getVoltage()/3.3*360) > 5) {

                // gets the current position of the arm
                currentPosition = HangMotor.getCurrentPosition();
                telemetry.addData("CurrentPosition", currentPosition);
                // telemetry.addData("Voltage", voltage);
                telemetry.update();
                target = target1;
                // calculates the power needed to move the arm with the feedforward constant
                double pid = controller.calculate(currentPosition, (double) (target * 4000) /360);
/*
            if (Math.abs(arm_encoder.getVoltage() / 3.3 * 360 - target) > 5) {
                pid = kp * (arm_encoder.getVoltage() / 3.3 * 360 - target);
                telemetry.addData("hang", arm_encoder.getVoltage() / 3.3 * 360 - target);
                telemetry.update();
            } else {
                pid = 0.;
            }

 */
                double encoderTicksInDegrees = 360;
                double ff = Math.cos(Math.toRadians((double) (target))) * kf;
                double power = pid + ff;
                //power = 0.2;
                //power = kp * (arm_encoder.getVoltage() / 3.3 * 360 - target);
                HangMotor.setPower(-power);
                HangMotor.setTargetPosition(target);
                HangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Math.abs(currentPosition - target) < 0.1) {
                    //HangMotor.setPower(0.);
                } else {
                    //  HangMotor.setPower(0.);
                }

                while(HangMotor.isBusy()) {
                    telemetry.addData("CurrentPosition_1",   HangMotor.getCurrentPosition());
                    //telemetry.addData("Voltage", voltage);
                    telemetry.addData("power1", pid);
                    telemetry.addData("power2", ff);
                    //telemetry.addData("hang2", arm_encoder.getVoltage()/3.3*360 - target);
                    telemetry.update();
                }




                pos_prev = currentPosition;
                // = arm_encoder.getVoltage();
                //currentPosition = (voltage - 0) / 3.3 * 360;

                // }
            }

            if (gamepad1.x) {
                kp = 0.001;
                ki = 0;
                kd = 0;
                kf=0.1;
                controller = new PIDController(kp, ki, kd);
                controller.setPID(kp, ki, kd);

                //double voltage = arm_encoder.getVoltage();
                // currentPosition = (voltage - 0) / 3.3 * 360;
                // pos_prev = 0.;
                // HangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//HangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // while ((pos_prev - currentPosition) != 0.) {
                telemetry.addData("Enter", 1.);
                telemetry.update();

                // gets the current position of the arm
                currentPosition = HangMotor.getCurrentPosition();

                telemetry.addData("CurrentPositionB", currentPosition);
                //  telemetry.addData("Voltage", voltage);
                telemetry.update();
                // calculates the power needed to move the arm with the feedforward constant
                target = target2;
                double pid = controller.calculate(currentPosition, (double) (target * 4000) /360);

                // if (Math.abs(currentPosition - (target)) < 7) {
                //  pid = 0.;
                //  } else {
                //  pid = 0.;
                // }
                //double encoderTicksInDegrees = 360;
                double ff = Math.cos(Math.toRadians((double) (target))) * kf;
                double power = pid + ff;

                HangMotor.setTargetPosition(target);
                HangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HangMotor.setPower(-power);
                if (Math.abs(currentPosition - target) < .25) {
                    //HangMotor.setPower(-power);
                    //} else {
                    //HangMotor.setPower(0.);
                }

                while (HangMotor.isBusy()) {
                    telemetry.addData("CurrentPosition", HangMotor.getCurrentPosition());

                    // telemetry.addData("Voltage", voltage);
                    telemetry.addData("power1", pid);
                    telemetry.addData("power2", ff);
                    telemetry.update();
                }
                // pos_prev = currentPosition;
                // voltage = arm_encoder.getVoltage();
                // currentPosition = (voltage - 0) / 3.3 * 360;
            }

        }

    }



}
//  }
//}
//}







