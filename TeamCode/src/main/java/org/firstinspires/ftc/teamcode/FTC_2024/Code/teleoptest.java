package org.firstinspires.ftc.teamcode.FTC_2024;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="teleoptest")
public class teleoptest extends LinearOpMode {
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor ExtentionMotor = null;
    DcMotor IntakeMotor = null;
    Servo IntakeServo = null;
    DcMotor LiftMotor = null;
    Servo BigWrist = null;
    Servo SmallPivot = null;
    //Servo SampleGrabber = null;

    DcMotor hangmotor = null;
    Servo latchServo = null;
    Servo slideServo = null;

    Servo SpecimanServo1 = null;

    Servo SpecimanServo2 = null;

    Servo SpecimanWrist = null;

    Servo SpecimanClaw=null;

    private DigitalChannel beamSensor;

    Orientation angles;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    @Override
    public void runOpMode() throws InterruptedException {

        FrontLeft = hardwareMap.dcMotor.get("FL")   ;
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        ExtentionMotor = hardwareMap.dcMotor.get("EM");
        IntakeMotor = hardwareMap.dcMotor.get("IM");
        IntakeServo = hardwareMap.servo.get("IS");
        LiftMotor = hardwareMap.dcMotor.get("LM");
        BigWrist = hardwareMap.servo.get("BW");
        // SmallPivot = hardwareMap.servo.get("SP");
        //   SampleGrabber = hardwareMap.servo.get("SG");
        // hangServo = hardwareMap.servo.get("HS1");
        // hangServo2 = hardwareMap.servo.get("HS2");
        hangmotor = hardwareMap.dcMotor.get("HM");
        latchServo = hardwareMap.servo.get("LS");
        slideServo = hardwareMap.servo.get("SS");
        SpecimanServo1 = hardwareMap.servo.get("ss1");
        SpecimanServo2 = hardwareMap.servo.get("ss2");
        SpecimanWrist = hardwareMap.servo.get("ssw");
        SpecimanClaw = hardwareMap.servo.get("ssc");
        beamSensor = hardwareMap.get(DigitalChannel.class, "beamSensor");
        beamSensor.setMode(DigitalChannel.Mode.INPUT);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same


        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ExtentionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        double slow_down_factor=0.5;
        double slow_down_factor2=1.;
        double extention_power = 0.9;
        double retraction_power = 0.9;
        int extention_target = 900;
        int retraction_target = 0;
        int retracion_target= 810;
        int extenion_target= 0;
        double hang_motor_power = 1;
        int extension_target_prev = 0;
        int extension_step=225;
        int extension_power_initial=1;

        String ExtentionMotorCurrentDirection = "extention";
        String LiftMotorCurrentDirection = "up";
        double servo_bucket_intake = 0.8;
        double servo_bucket_output = 0.09;
        //double servo_bucket_intake = 0.78; //old values for gobilda servos
        //double servo_bucket_output = 0.2;
        double intake_motor_power = 1;
        double intake_negative_power = -1;
        double lift_motor_power = 0.7;
        double big_wrist_position = -1;
        double bigout_wrist_position= 1;
        double small_pivot_position = 0.2  ;
        double sample_grabber_position = 0.6;
        double power_x=0.5;
        double power_y;
        double grabbing_position = 0.2;
        double smallout_pivot_position = -0.2;
        double hangposition=0.7;
        double climbposition=0.3;
        double latch_position = -1;
        double slideservo_position= 1;
        double specimanintake_position = 0.75;
        double specimanouttake_position = -1;
        double specimanclaw_position = 0.1;
        double specimanwrist_position = 0;

        double init_hang_pos;
        double ticks_per_rev = 537.7;
        int hang_target;
        double hang_rev_target = 2000;


        double y;


        FrontRight.setPower(0);
        BackLeft.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        ExtentionMotor.setPower(0);
        IntakeMotor.setPower(0);
        LiftMotor.setPower(0);
        //hangmotor.setPower(0);
        ExtentionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        ExtentionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Zero Power Behavior
        ExtentionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Zero Power Behavior
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        hangmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Zero Power Behavior
        hangmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime timer = new ElapsedTime(SECONDS);

        //code before waitForStart is run when Init button is pressed
        while (!isStarted()) {
            //print encoder counts to telemetry while we manually move the ar
            telemetry.addData("Extension motor position", ExtentionMotor.getCurrentPosition());
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            init_hang_pos = hangmotor.getCurrentPosition();
            hang_target = (int) (init_hang_pos + (int) hang_rev_target * ticks_per_rev / (360));
            y = -gamepad1.left_stick_y;
            telemetry.addData("Y", y);
            telemetry.update();
            //Strafe Right
            if (gamepad1.right_bumper) {

                FrontLeft.setPower(power_x);
                BackLeft.setPower(-power_x);
                FrontRight.setPower(-power_x);
                BackRight.setPower(power_x);
            }
            /*
            else {
                power_x = 0.;
                FrontLeft.setPower(power_x);
                BackLeft.setPower(-power_x);
                FrontRight.setPower(-power_x);
                BackRight.setPower(power_x);
            }*/
            //Strafe Left
            else if (gamepad1.left_bumper) {

                FrontLeft.setPower(-power_x);
                BackLeft.setPower(power_x);
                FrontRight.setPower(power_x);
                BackRight.setPower(-power_x);
            }
            /*
            else {
                power_x = 0.;
                FrontLeft.setPower(-power_x);
                BackLeft.setPower(power_x);
                FrontRight.setPower(power_x);
                BackRight.setPower(-power_x);
            }

             */

            if (y > 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }
            /*
            else {
                power_y = 0.;
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(-power_y);
                BackRight.setPower(-power_y);
            }*/
            if (y < 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }
            /*
            else {
                power_y = 0.;
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(-power_y);
                BackRight.setPower(-power_y);
            }*/
            //Turn antiClockwise
            if (gamepad1.right_stick_x > 0.0) {
                if (gamepad1.right_stick_y > 0.0) {
                    power_y = gamepad1.right_stick_y * slow_down_factor;
                } else {
                    power_y = gamepad1.right_stick_x * slow_down_factor;
                }

                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }
            /*
            else {
                power_y = 0.;
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(-power_y);
                BackRight.setPower(-power_y);
            }*/
            //Turn Clockwise
            if (gamepad1.right_stick_x < 0.0) {
                if (gamepad1.right_stick_y < 0.0) {
                    power_y = gamepad1.right_stick_y * slow_down_factor;
                } else {
                    power_y = gamepad1.right_stick_x * slow_down_factor;
                }
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }
            /*
            else {
                power_y = 0.;
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(-power_y);
                BackRight.setPower(-power_y);
            }*/
            if (gamepad1.y) {
                power_y = 1. * slow_down_factor2;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (gamepad1.a) {
                power_y = -1. * slow_down_factor2;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (!gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.right_stick_x == 0.0 && y==0. && !gamepad1.y && !gamepad1.a && gamepad1.left_trigger == 0.0 && gamepad1.right_trigger == 0.0){
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                FrontLeft.setPower(0.0);
                BackRight.setPower(0.0);
            }


            //For extention of the horizontal slides
            if (gamepad2.right_bumper) {
                extension_target_prev+=extension_step;
                if(extension_target_prev >= extention_target){
                    extension_target_prev=extention_target;
                }
                extension_target_prev=extention_target;
                extention_power = extension_power_initial;

                slideServo.setPosition(slideservo_position);

                safeWaitSeconds(0.5);
                ExtentionMotor.setPower(-extention_power);
                ExtentionMotor.setTargetPosition(extension_target_prev);
                ExtentionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ExtentionMotorCurrentDirection = "extention";

            }


            //For retraction of the horizontal slides
            else if (gamepad2.left_bumper) {
                retraction_power = extension_power_initial;
                ExtentionMotor.setPower(retraction_power);
                ExtentionMotor.setTargetPosition(retraction_target);
                ExtentionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ExtentionMotorCurrentDirection = "retraction";

                //telemetry.addData("Retraction",3);
                //telemetry.update();
            }

            //Remove  Power from the Extention Motor
            if (ExtentionMotorCurrentDirection.equals("extention") && (ExtentionMotor.getCurrentPosition() > extention_target)) {
                //extention_power = 0;
                //ExtentionMotor.setPower(extention_power);
                //ExtentionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //slideServo.setPosition(slideservo_position);
                telemetry.addData("Extension", 1);
                telemetry.update();
            }
            //Remove  Power from the Extention Motor
            if (ExtentionMotorCurrentDirection.equals("retraction") && (ExtentionMotor.getCurrentPosition() < retraction_target)) {
                extention_power = 0;
                ExtentionMotor.setPower(extention_power);
                ExtentionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideServo.setPosition(-slideservo_position);

                extension_target_prev=0;
                IntakeServo.setPosition(servo_bucket_intake);
                safeWaitSeconds(0.8);
                IntakeMotor.setPower(intake_motor_power);
                //telemetry.addData("Retraction",2);
                //telemetry.update();
            }
           // idle();

            //Extention Motor Telemetry
            if (ExtentionMotor.isBusy()) {
                telemetry.addData("Extension motor position", ExtentionMotor.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad2.right_trigger != 0.0) {

                IntakeServo.setPosition(servo_bucket_intake);

                safeWaitSeconds(0.8);


                IntakeMotor.setPower(intake_motor_power);

            }
            if (gamepad2.left_trigger != 0.0) {
                IntakeServo.setPosition(servo_bucket_output);
                safeWaitSeconds(0.8);

                IntakeMotor.setPower(-intake_motor_power);
                telemetry.addData("distance", sensorDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("Color", sensorColor.red());
                telemetry.update();
                if ((sensorDistance.getDistance(DistanceUnit.CM) <=5) && (sensorColor.red()>200)) {
                    IntakeMotor.setPower(-intake_motor_power);
                }
            }
            /*
            if (gamepad2.dpad_up)
                IntakeMotor.setPower(intake_motor_power);
            }
            if (gamepad1.left_trigger != 0.0) {
                FrontLeft.setPower(-power_x*5);
                BackLeft.setPower(power_x*5);
                FrontRight.setPower(power_x*5);
                BackRight.setPower(-power_x*5);
            }
            if (gamepad1.right_trigger!=0.0) {
                FrontLeft.setPower(power_x*5);
                BackLeft.setPower(-power_x*5);
                FrontRight.setPower(-power_x*5);
                BackRight.setPower(power_x*5);
            }
            if (gamepad2.dpad_left) {
                IntakeMotor.setPower(0);
            }

            if(gamepad1.b){
                SpecimanServo1.setPosition(specimanintake_position);
                SpecimanClaw.setPosition(specimanclaw_position);
                SpecimanWrist.setPosition(specimanwrist_position);


            }
            //make the intake motor spin when dpad down pressed
            if (gamepad2.dpad_down) {
                IntakeMotor.setPower(- intake_motor_power);

            }

            if (gamepad2.x) {

                LiftMotor.setPower(lift_motor_power);
                LiftMotor.setTargetPosition(extenion_target);
                LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftMotorCurrentDirection = "down";
            }

            if (gamepad2.y) {
                lift_motor_power = 0.7;
                LiftMotor.setPower(-lift_motor_power);
                LiftMotor.setTargetPosition(retracion_target);
                LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LiftMotorCurrentDirection = "up";
            }
            //Remove  Power from the lift Motor
            if (LiftMotorCurrentDirection.equals("up") && (LiftMotor.getCurrentPosition() > retracion_target)) {
                //extention_power = 0;
                //ExtentionMotor.setPower(extention_power);
                //ExtentionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("up", 1);
                telemetry.update();
            }
            //Remove  Power from the lift Motor
            if (LiftMotorCurrentDirection.equals("down") && (LiftMotor.getCurrentPosition() < extenion_target)) {
                lift_motor_power = 0;
                LiftMotor.setPower(lift_motor_power);
                LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //telemetry.addData("Retraction",2);
                //telemetry.update();
            }
            idle();

            //lift Motor Telemetry
            if (LiftMotor.isBusy()) {
                telemetry.addData("Lift motor position", LiftMotor.getCurrentPosition());
                telemetry.update();
            }
            if (gamepad2.a) {

                BigWrist.setPosition( big_wrist_position);


            }
            if (gamepad2.b) {
                BigWrist.setPosition(bigout_wrist_position);
            }
            // if (gamepad2.dpad_right) {
            //SampleGrabber.setPosition(grabbing_position);
            //  }

            if (gamepad1.dpad_up) {

                hangmotor.setPower(hang_motor_power);

                hangmotor.setTargetPosition(hang_target);

                hangmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.dpad_down) {
                hangmotor.setPower(-hang_motor_power);
                int hang_target2 = (int) (hangmotor.getCurrentPosition() - (int) hang_rev_target * ticks_per_rev / (360));
                hangmotor.setTargetPosition( hang_target2);
                hangmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

           /* if (gamepad2.right_stick_x>0) {
                latchServo.setPosition(latch_position);
            }
           else if (gamepad2.right_stick_x<0) {
                latchServo.setPosition(-latch_position);
            }


            if (gamepad2.left_stick_x>0) {
                slideServo.setPosition(slideservo_position);
            }
            else if (gamepad2.left_stick_x<0) {
                slideServo.setPosition(-slideservo_position);
            }


            if (beamSensor.getState()) {
                // Beam is NOT broken, sensor is reading HIGH
                telemetry.addData("Beam Status", "Not Broken");
            } else {
                // Beam is broken, sensor is reading LOW
                IntakeMotor.setPower(0);

                telemetry.addData("Beam Status", "  Broken");
            }

 */

        }
    }
    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        double power_x=0.5;
        double power_y,y;
        double slow_down_factor=0.5;
        double slow_down_factor2=1.;
        y = -gamepad1.left_stick_y;
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //Strafe Right
            if (gamepad1.right_bumper) {

                FrontLeft.setPower(power_x);
                BackLeft.setPower(-power_x);
                FrontRight.setPower(-power_x);
                BackRight.setPower(power_x);
            }
            /*
            else {
                power_x = 0.;
                FrontLeft.setPower(power_x);
                BackLeft.setPower(-power_x);
                FrontRight.setPower(-power_x);
                BackRight.setPower(power_x);
            }*/
            //Strafe Left
            else if (gamepad1.left_bumper) {

                FrontLeft.setPower(-power_x);
                BackLeft.setPower(power_x);
                FrontRight.setPower(power_x);
                BackRight.setPower(-power_x);
            }
            /*
            else {
                power_x = 0.;
                FrontLeft.setPower(-power_x);
                BackLeft.setPower(power_x);
                FrontRight.setPower(power_x);
                BackRight.setPower(-power_x);
            }

             */

            if (y > 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }
            /*
            else {
                power_y = 0.;
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(-power_y);
                BackRight.setPower(-power_y);
            }*/
            if (y < 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }
            /*
            else {
                power_y = 0.;
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(-power_y);
                BackRight.setPower(-power_y);
            }*/
            //Turn antiClockwise
            if (gamepad1.right_stick_x > 0.0) {
                if (gamepad1.right_stick_y > 0.0) {
                    power_y = gamepad1.right_stick_y * slow_down_factor;
                } else {
                    power_y = gamepad1.right_stick_x * slow_down_factor;
                }

                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }
            /*
            else {
                power_y = 0.;
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(-power_y);
                BackRight.setPower(-power_y);
            }*/
            //Turn Clockwise
            if (gamepad1.right_stick_x < 0.0) {
                if (gamepad1.right_stick_y < 0.0) {
                    power_y = gamepad1.right_stick_y * slow_down_factor;
                } else {
                    power_y = gamepad1.right_stick_x * slow_down_factor;
                }
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }
            /*
            else {
                power_y = 0.;
                FrontLeft.setPower(-power_y);
                BackLeft.setPower(-power_y);
                FrontRight.setPower(-power_y);
                BackRight.setPower(-power_y);
            }*/
            if (gamepad1.y) {
                power_y = 1. * slow_down_factor2;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (gamepad1.a) {
                power_y = -1. * slow_down_factor2;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (!gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.right_stick_x == 0.0 && y == 0. && !gamepad1.y && !gamepad1.a && gamepad1.left_trigger == 0.0 && gamepad1.right_trigger == 0.0) {
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                FrontLeft.setPower(0.0);
                BackRight.setPower(0.0);
            }

        }
    }
}


