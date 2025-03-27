package org.firstinspires.ftc.teamcode.FTC_2024.Code;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name="Integrating_Pid_with_teleop")
//@Config //enables tuning through the FTC dashboard
public class Integrating_Pid_with_teleop extends LinearOpMode {
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor IntakeMotor = null;
    Servo IntakeServo = null;
    DcMotorEx LiftMotor = null;
    DcMotorEx ExtensionMotor = null;
    Servo BigWrist = null;
    DcMotor hangmotor = null;
    Servo latchServo = null;
    Servo slideServo = null;

    Servo SweeperServo = null;

    Servo SpecimenServo1 = null;

    Servo SpecimenServo2 = null;

    Servo SpecimenWrist = null;

    Servo SpecimenClaw=null;
    DistanceSensor distance=null ;

    private DigitalChannel beamSensor;

    // define the proportional, integral, derivative, feedforward for lift
    public static double Kp_lift = 0.0;
    public static double Ki_lift = 0.0; // to tune: use the tecniques from control ftc
    public static double Kd_lift = 0.0;
    public static double Kf_lift = 0.075; //  the feedforward component and does not rely on measurments
    // start with Kf as small value and then Kp

    //PIDF variables for lift
    public static double integralSum_lift = 0;
    public static double lastError_lift = 0;

    //define the terms for extension
    public static double Kp_extension = 0.0;
    public static double Ki_extension = 0.0; // to tune: use the tecniques from control ftc
    public static double Kd_extension = 0.0;
    public static double Kf_extension = 0.075; //  the feedforward component and does not rely on measurments

    //PIDF variables for extension
    public static double integralSum_extension = 0;
    public static double lastError_extension = 0;

    //timer for PIDF loops for the lift
    ElapsedTime liftTimer = new ElapsedTime(SECONDS);

    //timer for PIDF loops for the extension
    ElapsedTime extensionTimer = new ElapsedTime(SECONDS);

    //timer for the intake servo
    ElapsedTime intakeTimer = new ElapsedTime(SECONDS);

    // Set the target positions for the Lift
    private static final int retractedPosition_lift = 3;
    private static final int extendedPosition_lift = 810;
    private int currentTargetPosition_lift = retractedPosition_lift; // Start retracted

    // Set the different lift states
    enum LiftState {
        EXTENDING, RETRACTING, HOLDING
    }

    LiftState currentLiftState = LiftState.HOLDING;

    //Set target positions for extension
    private static final int retractedPosition_extension = 1;
    private static final int extendedPosition_extension = 225;
    private int currentTargetPosition_extension = retractedPosition_extension; // Start retracted

    // Set the different extension states
    enum ExtensionState {
        EXTENDING, RETRACTING, HOLDING
    }

    ExtensionState currentExtensionState = ExtensionState.HOLDING;



    @Override
    public void runOpMode() throws InterruptedException {

        //get the FTC dashboard
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        FrontLeft = hardwareMap.dcMotor.get("FL")   ;
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        IntakeMotor = hardwareMap.dcMotor.get("IM");
        IntakeServo = hardwareMap.servo.get("IS");
        LiftMotor = hardwareMap.get(DcMotorEx.class, "LM");
        ExtensionMotor = hardwareMap.get(DcMotorEx.class, "EM");

        BigWrist = hardwareMap.servo.get("BW");

        hangmotor = hardwareMap.dcMotor.get("HM");
        latchServo = hardwareMap.servo.get("LS");
        slideServo = hardwareMap.servo.get("SS");
        SpecimenServo1 = hardwareMap.servo.get("ss1");
        SpecimenServo2 = hardwareMap.servo.get("ss2");
        SpecimenWrist = hardwareMap.servo.get("ssw");
        SpecimenClaw = hardwareMap.servo.get("ssc");
        beamSensor = hardwareMap.get(DigitalChannel.class, "beamSensor");
        beamSensor.setMode(DigitalChannel.Mode.INPUT);
        // Get the distance sensor and motor from hardwareMap
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        IntakeMotor = hardwareMap.dcMotor.get("IM");
        SweeperServo = hardwareMap.servo.get("SweeperServo");


        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double slow_down_factor=0.5;
        double slow_down_factor2=1.;
        double hang_motor_power = 1;
        double servo_bucket_intake = 0.8;
        double servo_bucket_output = 0.09;
        double servo_in=0;
        double servo_out=1;
        double intake_motor_power = 1;
        double power_x=0.5;
        double power_y;
        double Specimenintake_position = 0.75;
        double Specimenclaw_position = 0.1;
        double Specimenwrist_position = 0;

        double init_hang_pos;
        double ticks_per_rev = 537.7;
        int hang_target;
        double hang_rev_target = 2000;

        double y;


        FrontRight.setPower(0);
        BackLeft.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        IntakeMotor.setPower(0);
        //hangmotor.setPower(0);

        //set the encoders and directions for the Lift
        LiftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set the encoders and directions for the Extension
        ExtensionMotor.setDirection(DcMotorEx.Direction.FORWARD);
        ExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set encoders to 0
        hangmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Zero Power Behavior
        hangmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //wait for start
        waitForStart();

        //code before waitForStart is run when Init button is pressed
        while (!isStarted()) {
            telemetry.addData("Lift Status: ", currentLiftState); //for the lift
            telemetry.addData("Extension Status: ", currentExtensionState); //for the extension
            telemetry.update(); //update the telemetry
        }

        while (opModeIsActive()) {
            controlLift(); // call the function which controls the lift
            controlExtension(); // call the function which controls the extension

            init_hang_pos = hangmotor.getCurrentPosition();
            hang_target = (int) (init_hang_pos + (int) hang_rev_target * ticks_per_rev*0.9/ (360));
            y = -gamepad1.left_stick_y;
            telemetry.addData("Y", y);
            telemetry.update(); // update the telemetry in the main loop

            //Strafe Right
            if (gamepad1.right_bumper) {

                FrontLeft.setPower(power_x);
                BackLeft.setPower(-power_x);
                FrontRight.setPower(-power_x);
                BackRight.setPower(power_x);
            }

            else if (gamepad1.left_bumper) {

                FrontLeft.setPower(-power_x);
                BackLeft.setPower(power_x);
                FrontRight.setPower(power_x);
                BackRight.setPower(-power_x);
            }

            if (y > 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (y < 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

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

            if (gamepad2.right_trigger != 0.0) {
                IntakeServo.setPosition(servo_bucket_intake);
                intakeTimer.reset();
                safeWaitSeconds(0.8);
                IntakeMotor.setPower(intake_motor_power);
            }

            if (gamepad2.left_trigger != 0.0) {
                IntakeServo.setPosition(servo_bucket_output);
                safeWaitSeconds(0.8);
                IntakeMotor.setPower(-intake_motor_power);
            }

            if (gamepad2.dpad_up) {
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
                SpecimenServo1.setPosition(Specimenintake_position);
                SpecimenClaw.setPosition(Specimenclaw_position);
                SpecimenWrist.setPosition(Specimenwrist_position);
            }

            //make the intake motor spin when dpad down pressed
            if (gamepad2.dpad_down) {
                IntakeMotor.setPower(-intake_motor_power);
            }


            if (gamepad1.dpad_up) {

                hangmotor.setPower(hang_motor_power);

                hangmotor.setTargetPosition(hang_target);

                hangmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.dpad_down) {
                hangmotor.setPower(-hang_motor_power);
                int hang_target2 = (int) (hangmotor.getCurrentPosition() - (int) hang_rev_target * ticks_per_rev/ (360));
                hangmotor.setTargetPosition( hang_target2);
                hangmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // If the distance in centimeters is less than 10, set the power to 0.5
            if (distance.getDistance(DistanceUnit.CM) < 2.3) {
                IntakeMotor.setPower(-0.5);
            }

            if (gamepad2.dpad_right) {
                SweeperServo.setPosition(servo_in);
                safeWaitSeconds(0.8);
                SweeperServo.setPosition(servo_out);
            }

            if (beamSensor.getState()) {
                // Beam is NOT broken, sensor is reading HIGH
                telemetry.addData("Beam Status", "Not Broken");
            } else {
                // Beam is broken, sensor is reading LOW
                IntakeMotor.setPower(0);

                telemetry.addData("Beam Status", "  Broken");
            }

        }
    }

    // main PIDF loop

    private void controlLift() {
        //reset timer
        liftTimer.reset();

        // get the input from the gamepad for the lift
        if (gamepad2.x && !gamepad2.y) { // for the lift is gpad2.x is pressed
            currentTargetPosition_lift = extendedPosition_lift; //set the target position
            currentLiftState = LiftState.EXTENDING; // set the state to extending
        } else if (gamepad2.y && !gamepad2.x) { // if gpad2.y is pressed
            currentTargetPosition_lift = retractedPosition_lift; //set target position to the retracted position
            currentLiftState = LiftState.RETRACTING; // set the state to retracting
        } else {
            currentLiftState = LiftState.HOLDING; // if not pressed, use feedforward and hold position
        }

        // finding the power
        double currentPosition_lift = LiftMotor.getCurrentPosition();
        double error_lift = currentTargetPosition_lift - currentPosition_lift;
        double dt_lift = liftTimer.seconds();
        double derivative_lift = (error_lift - lastError_lift) / dt_lift;
        integralSum_lift += error_lift * dt_lift;

        //final power
        double output_lift = (Kp_lift * error_lift) + (Ki_lift * integralSum_lift) + (Kd_lift * derivative_lift) + Kf_lift;

        // Set power to the motor
        LiftMotor.setPower(output_lift);

        // Update the error so that it will work in the next loop
        lastError_lift = error_lift;

        // update the telemetry
        //TelemetryPacket packet = new TelemetryPacket();
        telemetry.addData("Lift Status: ", currentLiftState);
        telemetry.addData("Target Position for Lift: ", currentTargetPosition_lift);
        telemetry.addData("Power of Lift: ", output_lift);
        //send packet to the dashboard
        //dashboard.sendTelemetryPacket(packet);
    }

    //stay idle
    //void idle();

    private void controlExtension() {
        //reset timer
        extensionTimer.reset();

        // set position for intake
        double servo_bucket_intake = 0.8;

        //get input fromt gamepad for the lift
        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            currentTargetPosition_extension = extendedPosition_extension; //set the target position
            currentExtensionState = ExtensionState.EXTENDING; // set the state to extending
        } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            IntakeServo.setPosition(servo_bucket_intake);
            currentTargetPosition_extension = retractedPosition_extension; //set target position to the retracted position
            currentExtensionState = ExtensionState.RETRACTING; // set the state to retracting
        } else {
            currentExtensionState = ExtensionState.HOLDING; // if not pressed, use feedforward and hold position
        }

        // finding the power
        int currentPosition_extension = ExtensionMotor.getCurrentPosition();
        double error_extension = currentTargetPosition_extension - currentPosition_extension;
        double dt_extension = extensionTimer.seconds();
        double derivative_extension = (error_extension - lastError_extension) / dt_extension;
        integralSum_extension += error_extension * dt_extension;

        //final power
        double output_extension = (Kp_extension * error_extension) + (Ki_extension * integralSum_extension) + (Kd_extension * derivative_extension) + Kf_extension;

        // Set power to the motor
        ExtensionMotor.setPower(output_extension);

        // set slide servo position
        double slideservo_position= 1;

        if (currentPosition_extension == retractedPosition_extension) {
            slideServo.setPosition(-slideservo_position);
        } else if (currentExtensionState == ExtensionState.EXTENDING) {
            slideServo.setPosition(slideservo_position);
        } else {
            slideServo.setPosition(slideservo_position);
        }

        // Update the error so that it will work in the next loop
        lastError_extension = error_extension;

        // update the telemetry
        //TelemetryPacket packet = new TelemetryPacket();
        telemetry.addData("Extension Position: ", currentPosition_extension);
        telemetry.addData("Target Position for Extension: ", currentTargetPosition_extension); // Corrected line
        telemetry.addData("Power of Extension: ", output_extension);
        //send packet to the dashboard
        //dashboard.sendTelemetryPacket(packet);
    }

    //stay idle
    //void idle();

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        double power_x=0.5;
        double power_y,y;
        double slow_down_factor=0.5;
        double slow_down_factor2=1.;
        y = -gamepad1.left_stick_y;
        ElapsedTime safewaitTimer = new ElapsedTime(SECONDS);
        safewaitTimer.reset();
        while (!isStopRequested() && safewaitTimer.time() < time) {
            //Strafe Right
            if (gamepad1.right_bumper) {
                FrontLeft.setPower(power_x);
                BackLeft.setPower(-power_x);
                FrontRight.setPower(-power_x);
                BackRight.setPower(power_x);
            }

            else if (gamepad1.left_bumper) {
                FrontLeft.setPower(-power_x);
                BackLeft.setPower(power_x);
                FrontRight.setPower(power_x);
                BackRight.setPower(-power_x);
            }

            if (y > 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

            if (y < 0.0) {
                power_y = y * slow_down_factor;
                FrontLeft.setPower(power_y);
                BackLeft.setPower(power_y);
                FrontRight.setPower(power_y);
                BackRight.setPower(power_y);
            }

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

