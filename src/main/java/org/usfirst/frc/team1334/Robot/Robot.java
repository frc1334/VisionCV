package org.usfirst.frc.team1334.Robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1334.Robot.Commands.DriveCommand;
import org.usfirst.frc.team1334.Robot.Subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.usfirst.frc.team1334.Robot.Subsystems.PickupSubsystem;
import org.usfirst.frc.team1334.Robot.Subsystems.ShooterSubsystem;
import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.videoio.*;

import org.usfirst.frc.team1334.Vision.Vision;

/**
 * Created by FRC1334 on 8/15/2016.
 */
public class Robot extends IterativeRobot {
    public static DriveSubsystem DRIVE_SUBSYSTEM;
    public static PickupSubsystem PICKUP_SUBSYSTEM;
    public static ShooterSubsystem SHOOTER_SUBSYSTEM;
    double CAMERA_EXPOSURE = 0.0;
    //Image frame;
    //int session;
    public NetworkTable nettable;
    public static OI oi;
    //VideoCapture capture;
    Command DriveCommands;
    SendableChooser chooser;

    Vision vt;

    @Override
    public void robotInit() {
        // make a 7x7 complex matrix filled with 1+3j.
        //Mat M(7,7,CV_32FC2,Scalar(1,3));
        // and now turn M to a 100x60 15-channel 8-bit matrix.
        // The old content will be deallocated
        //M.create(100,60,CV_8UC(15));
        //Scalar s = Mat(1,1);


        //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        DRIVE_SUBSYSTEM = new DriveSubsystem ();
        PICKUP_SUBSYSTEM = new PickupSubsystem();
        SHOOTER_SUBSYSTEM = new ShooterSubsystem();
        //nettable = NetworkTable.getTable("GRIP/myContoursReport");

        Robot.DRIVE_SUBSYSTEM.left1.setControlMode(0);
        Robot.DRIVE_SUBSYSTEM.left2.setControlMode(0);
        Robot.DRIVE_SUBSYSTEM.right1.setControlMode(0);
        Robot.DRIVE_SUBSYSTEM.right2.setControlMode(0);


        oi = new OI();
        DRIVE_SUBSYSTEM = new DriveSubsystem();
        DriveCommands = new DriveCommand();
        chooser = new SendableChooser();

        /*vt = new Vision();
        vt.init_params();
        vt.init_extrinsics();*/

        /*frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // the camera name (ex "cam0") can be found through the roborio web interface
        session = NIVision.IMAQdxOpenCamera("cam1",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);*/
        SmartDashboard.putData("Auto Mode", chooser);
        super.robotInit();
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        super.autonomousPeriodic();
    }

    @Override
    public void teleopInit() {
        DriveCommands.start();
        /*capture = new VideoCapture();
        if (!capture.set(Videoio.CAP_PROP_FRAME_WIDTH, 1280));
        if (!capture.set(Videoio.CAP_PROP_FRAME_HEIGHT, 720));
        capture.set(Videoio.CAP_PROP_EXPOSURE, CAMERA_EXPOSURE);*/
        super.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        //long startTime, endTime;
        Scheduler.getInstance().run();
        super.teleopPeriodic();
        //Mat captured = new Mat();

       /* startTime = System.currentTimeMillis();
        capture.read(captured);
        endTime = System.currentTimeMillis();
        System.out.println("Frame capture time: " + (endTime - startTime) );

        startTime = System.currentTimeMillis();
        try {
            vt.set_image(captured);
            vt.extractContours();
            vt.filterContours();
            vt.extractRoughCorners();
            vt.refineCorners();
            vt.solvePnP();
        } catch(Exception e) {
            System.out.println(e.toString());
            e.printStackTrace();
        }
        endTime = System.currentTimeMillis();
        System.out.println("Total execution time: " + (endTime - startTime) );*/

        /*double[] defaultValue = new double[0];
        double[] centerX = nettable.getNumberArray("centerX", defaultValue);
        double[] x1 = nettable.getNumberArray("x1", defaultValue);
        double[] x2 = nettable.getNumberArray("x2", defaultValue);
        double[] y1 = nettable.getNumberArray("y1", defaultValue);
        double[] y2 = nettable.getNumberArray("y2", defaultValue);
        double[] length = nettable.getNumberArray("length",defaultValue);
        double[] angle = nettable.getNumberArray("angle",defaultValue);


        for(double centerx: centerX){
            System.out.print(centerx + ", ");
        }*/


        /*NIVision.IMAQdxStartAcquisition(session);


        NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);

        while (isOperatorControl() && isEnabled()) {

            NIVision.IMAQdxGrab(session, frame, 1);
            NIVision.imaqDrawShapeOnImage(frame, frame, rect,
                    DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);

            CameraServer.getInstance().setImage(frame);


            Timer.delay(0.005);		// wait for a motor update time
        }
        NIVision.IMAQdxStopAcquisition(session);*/
    }
}
