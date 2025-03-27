package org.firstinspires.ftc.teamcode.FTC_2024;//package org.firstinspires.ftc.teamcode.opmode.example;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Robotconstants;
import org.firstinspires.ftc.teamcode.config.subsystem.extensionsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.subsystem.liftsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.intakesystem;
import org.firstinspires.ftc.teamcode.config.subsystem.outtakesystem;
import com.qualcomm.robotcore.hardware.DigitalChannel;


/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "PedroPath_Regionals", group = "Examples")
public class Pedropath_Regionals extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    //HardwareMap hardwareMap;
    private liftsystem lift;
    public intakesystem intake;
    public outtakesystem outtake;
    public extensionsystem extension;
    public Servo BigWrist;
    public Servo IntakeServo;
    DigitalChannel breakBeamSensor;
    Servo slideServo = null;
    Servo hangServo1 = null;



    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(0, 111 );

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    //private final Pose scorePose = new Pose(14, 129, Math.toRadians(45));
    private final Pose scorePose = new Pose(7, 130, Math.toRadians(-45));
    //private final Pose scorePose = new Pose(19, 111);

    /** Lowest (First) Sample from the Spike Mark */
    //private final Pose pickup1Pose = new Pose(23, 128);
    private final Pose pickup1Pose = new Pose(9, 131, Math.toRadians(-35));
    /** Lowest (First) Sample from the Spike Mark */
    private final Pose intake1Pose = new Pose(10, 128);

    private final Pose score1Pose = new Pose(12, 138, Math.toRadians(-55));

    //private final Pose intake2Pose = new Pose(10, 140);

    /** Middle (Second) Sample from the Spike Mark */
    //private final Pose pickup2Pose = new Pose(29, 141);
    private final Pose intake2Pose = new Pose(10, 138);

    /** Middle (Second) Sample from the Spike Mark */
    //private final Pose pickup2Pose = new Pose(21, 138);
    private final Pose pickup2Pose = new Pose(14, 138);

    private final Pose score2Pose = new Pose(14, 139, Math.toRadians(-65));

    /** Highest (Third) Sample from the Spike Mark */
    //private final Pose pickup3Pose = new Pose(19  , 134, Math.toRadians(50));
    private final Pose pickup3Pose = new Pose(16, 139, Math.toRadians(50));
    private final Pose intake3Pose = new Pose(21  , 125, Math.toRadians(65));
    //private final Pose score3Pose = new Pose(9, 136, Math.toRadians(-60));
    private final Pose score3Pose = new Pose(12, 139, Math.toRadians(-65));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(50, 90, Math.toRadians(-90));

    private final Pose parkPose1 = new Pose(55, 80, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(40, 120);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain park1,park,grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, intakePickup1,intakePickup2,intakePickup3;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        //scorePreload.setConstantInterpolation(startPose.getHeading());
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        //scorePreload.setTangentHeadingInterpolation();


        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /*grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose), new Point(pickup1Pose)))
                //.setTangentHeadingInterpolation()
                //.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();

         */

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                //.setLinearHeadingInterpolation(intake1Pose.getHeading(), pickup1Pose.getHeading())
                .build();


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        intakePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(intake1Pose)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading())
                //.setTangentHeadingInterpolation()
                .build();

        intakePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(intake2Pose)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(score1Pose.getHeading(), intake2Pose.getHeading())
                .build();


        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
       /* grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        */

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score1Pose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(score2Pose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score2Pose), new Point(pickup3Pose)))
                //.setConstantHeadingInterpolation(pickup3Pose.getHeading())
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        intakePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(intake3Pose)))
                //.setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), intake3Pose.getHeading())
                .build();


        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake3Pose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), score3Pose.getHeading())
                .build();*/

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(score3Pose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
       // park = new Path(new BezierCurve(new Point(score3Pose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        //park.setLinearHeadingInterpolation(score3Pose.getHeading(), parkPose.getHeading());

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score3Pose), new Point(parkPose)))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), parkPose.getHeading())
                .build();

        park1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(parkPose), new Point(parkPose1)))
                .setLinearHeadingInterpolation(parkPose.getHeading(), parkPose1.getHeading())
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        breakBeamSensor = hardwareMap.get(DigitalChannel.class, "beamSensor");
        breakBeamSensor.setMode(DigitalChannel.Mode.INPUT);
        switch (pathState) {
            case 0:
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                //telemetry.addData("Encoder1:", leftFront.getCurrentPosition());
                //telemetry.addData("Encoder2:", leftRear.getCurrentPosition());
                //telemetry.addData("Encoder3:", rightFront.getCurrentPosition());
                //telemetry.addData("Encoder4:", rightRear.getCurrentPosition());
                follower.setMaxPower(0.95);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */


                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                //if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                if(!follower.isBusy()) {
                    /* Score Preload */
                    //follower.breakFollowing();
                    // boolean beamBroken = breakBeamSensor.getState(); // Inverted logic, true when beam is broken

                    // if (!beamBroken) {
                    // telemetry.addData("Beam Status", "Broken");
                    // telemetry.update();
                    lift.elevate(0, outtake, opmodeTimer.getElapsedTime(),extension);
                    // }
                    // else {
                    //    telemetry.addData("Beam Status", "not Broken");
                    //     telemetry.update();
                    // }

                    //follower.update();

                    //Path sample1pickupPose = new Path(new BezierLine(new Point(scorePose), new Point(pickup1Pose))).setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading());
                    //follower.setMaxPower();
                    //follower.setMaxPower(0.8);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    //follower.followPath(intakePickup1, true);
                    setPathState(2);
                }


                break;

            case 2:
                //follower.setMaxPower(0.8);
                //if(pathTimer.getElapsedTimeSeconds() < 2) {
                //intake.grab(pathTimer);
                //}
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                // if(follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {
                /* Grab Sample */
                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                   // intake.grab(pathTimer);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intake.grab(pathTimer);
                    slideServo.setPosition(Robotconstants.slideservo_position);
                    // intake.grab(pathTimer);
                    extension.extend(1);
                    safeWaitSeconds(0.3);
                    extension.extend(2);
                    follower.followPath(scorePickup1, true);
                   // follower.followPath(grabPickup1, true);
                    setPathState(4);
                }

                break;
          /*  case 3:

                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    intake.grab(pathTimer);
                    follower.followPath(scorePickup1, true);
                    setPathState(4);
                }
                break;

           */
            case 4:

                //while(follower.isBusy()){
                //intake.grab();
                // }
                // if(pathTimer.getElapsedTimeSeconds() < 2) {
                //outtake.grab(1);
                //}
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                // if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                if(!follower.isBusy()) {
                    //breakBeamSensor.setState(true);

                    boolean beamBroken = outtake.grab(1);
                    // breakBeamSensor.setState(true);

                    /* Score Sample */
                    // follower.breakFollowing();


                    telemetry.addData("Beam Status1", beamBroken);

                    // if (!beamBroken) {
                    //  telemetry.addData("Beam Status", "Broken");
                    // telemetry.update();
                    lift.elevate(1, outtake, opmodeTimer.getElapsedTime(),extension);
                    // }
                    //    else {
                    //     telemetry.addData("Beam Status", "not Broken");
                    //      telemetry.update();
                    // }

                    //outtake.grab(2);

                    //follower.update();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(intakePickup2,true);
                    follower.followPath(grabPickup2,true);
                    setPathState(5);
                }
                break;
            /*case 5:


                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    intake.grab(pathTimer);


                    //follower.followPath(scorePickup1, true);
                    follower.followPath(grabPickup2, true);
                    setPathState(6);
                }
                break;

             */
            case 5:
                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    //intake.grab(pathTimer);
                    intake.grab(pathTimer);
                    slideServo.setPosition(Robotconstants.slideservo_position);
                    // intake.grab(pathTimer);
                    extension.extend(1);
                    safeWaitSeconds(0.3);
                    extension.extend(2);
                    //boolean beamBroken = outtake.grab(1);
                    //follower.followPath(scorePickup3, true);
                    //follower.followPath(scorePickup3, true);
                    follower.followPath(scorePickup2, true);
                    setPathState(7);
                }
                break;
            case 7:

                if(!follower.isBusy()) {
                    boolean beamBroken = outtake.grab(1);
                    /* Score Sample */
                    // follower.breakFollowing();
                    // if(!beamBroken) {
                    lift.elevate(1, outtake, opmodeTimer.getElapsedTime(),extension);
                    //  }
                    //else
                    // {

                    //}
                    //outtake.grab(2);

                    //follower.update();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(8);
                }
                break;
            case 8:
                //telemetry.addData("follower", follower.isBusy());
               // telemetry.update();

                if(!follower.isBusy()) {
                    //follower.breakFollowing();
                    intake.grab(pathTimer);
                    slideServo.setPosition(Robotconstants.slideservo_position);
                   // intake.grab(pathTimer);
                    extension.extend(1);
                    safeWaitSeconds(0.3);
                    extension.extend(2);
                    slideServo.setPosition(-1);
                    //boolean beamBroken = outtake.grab(1);
                    //follower.followPath(scorePickup3, true);
                    follower.followPath(scorePickup3, true);
                    //follower.followPath(scorePickup3, true);
                    setPathState(9);
                }
                break;
            /*case 9:
                //follower.breakFollowing();
                if(follower.isBusy()) {
                    //follower.breakFollowing();
                    intake.grab(pathTimer);
                    extension.extend(1);

                    extension.extend(2);
                    //follower.followPath(scorePickup3, true);
                    follower.followPath(scorePickup3, true);
                    setPathState(10);
                }
                break;

             */
            case 9:
                //if(pathTimer.getElapsedTimeSeconds() < 2) {
                    // boolean beamBroken =  outtake.grab(1);
               // }
                if(!follower.isBusy()) {
                    //outtake.grab(1);
                    /* Score Sample */
                    // follower.breakFollowing();
                    boolean beamBroken =  outtake.grab(1);
                    //if(!beamBroken) {
                    lift.elevate(1, outtake, opmodeTimer.getElapsedTime(),extension);
                    //}
                    // else
                    //{

                    // }
                    //outtake.grab(2);

                    //follower.update();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(park,true);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
               // if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
                    /* Put the claw in position to get a level 1 ascent */
                    ////follower.setMaxPower(0.);
                    //hangServo1.setPosition(0.7);
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                follower.followPath(park1,true);
                    setPathState(12);
               // }
                break;

        case 12:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
        //if(follower.getPose().getX() > (parkPose1.getX() - 1) && follower.getPose().getY() > (parkPose1.getY() - 1))
            if(!follower.isBusy()){
            /* Put the claw in position to get a level 1 ascent */
            follower.setMaxPower(0.);
            hangServo1.setPosition(1);
            /* Set the state to a Case we won't use or define, so it just stops running an new paths */
            setPathState(-1);
        }
        break;
    }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (timer.time() < time) {
        }
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        breakBeamSensor = hardwareMap.get(DigitalChannel.class, "beamSensor");
        breakBeamSensor.setMode(DigitalChannel.Mode.INPUT);

        // These loop the movements of the robot
        follower.update();

        autonomousPathUpdate();


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        lift = new liftsystem(hardwareMap);
        intake = new intakesystem(hardwareMap);
        outtake = new outtakesystem(hardwareMap);
        extension = new extensionsystem(hardwareMap);
        slideServo = hardwareMap.servo.get("SS");
        slideServo.setPosition(-1);
        IntakeServo= hardwareMap.servo.get("IS");
        IntakeServo.setPosition(0.65);
        hangServo1 = hardwareMap.servo.get("hangservo1");


        BigWrist = hardwareMap.get(Servo.class, "BW");
        BigWrist.setPosition(Robotconstants.servoposition);

        breakBeamSensor = hardwareMap.get(DigitalChannel.class, "beamSensor");
        breakBeamSensor.setMode(DigitalChannel.Mode.INPUT); // Set the sensor mode to input

        boolean beamBroken = breakBeamSensor.getState(); // Inverted logic, true when beam is broken
        telemetry.addData("Beam Status1", beamBroken);

        if (!beamBroken) {
            telemetry.addData("Beam Status", "Broken");
            telemetry.update();

        }
        else {
            telemetry.addData("Beam Status", "not Broken");
            telemetry.update();
        }

        buildPaths();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
