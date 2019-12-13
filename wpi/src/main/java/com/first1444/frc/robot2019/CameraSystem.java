package com.first1444.frc.robot2019;

import com.first1444.dashboard.shuffleboard.ComponentMetadataHelper;
import com.first1444.dashboard.shuffleboard.SendableComponent;
import com.first1444.dashboard.wpi.VideoSourceSendable;
import com.first1444.frc.robot2019.subsystems.TaskSystem;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotBase;
import me.retrodaredevil.action.SimpleAction;

import java.util.function.Supplier;

import static java.util.Objects.requireNonNull;

public class CameraSystem extends SimpleAction {
    private static final VideoMode VIDEO_MODE = new VideoMode(VideoMode.PixelFormat.kMJPEG, 280, 210, 11);
    /** The compression level. A number between 0 and 100. The lower the value, the more compressed it is.*/
    private static final int COMPRESSION_LEVEL = -1;
    private final Supplier<TaskSystem> taskSystemSupplier;
    private final UsbCamera hatch;
    private final UsbCamera cargo;
    private final MjpegServer videoSink;
    private TaskSystem.Task lastTask = null;

    /**
     * @param dashboardMap The shuffleboard map to add the camera to
     * @param taskSystemSupplier The supplier for the {@link TaskSystem}. This will not be called by the constructor
     */
    public CameraSystem(DashboardMap dashboardMap, Supplier<TaskSystem> taskSystemSupplier) {
        super(false);
        this.taskSystemSupplier = taskSystemSupplier;

        if(RobotBase.isSimulation()){
            // These values are for your own computer. If you want to simulate something in the future, change these for yourself.
            hatch = CameraServer.getInstance().startAutomaticCapture(0);
            cargo = CameraServer.getInstance().startAutomaticCapture(2);
        } else {
            // Values for the cameras on the roborio
            hatch = CameraServer.getInstance().startAutomaticCapture(0);
            cargo = CameraServer.getInstance().startAutomaticCapture(1);
        }
        setupCamera(hatch);
        setupCamera(cargo);

        videoSink = CameraServer.getInstance().addSwitchedCamera("Toggle Camera");
        final VideoSource source = videoSink.getSource();
        if(COMPRESSION_LEVEL >= 0) {
            videoSink.setCompression(COMPRESSION_LEVEL);
            videoSink.setDefaultCompression(COMPRESSION_LEVEL);
        }
        dashboardMap.getUserTab().add("My Toggle Camera", new SendableComponent<>(new VideoSourceSendable("camera_server://" + source.getName())),
            (metadata) -> new ComponentMetadataHelper(metadata).setSize(7, 5).setPosition(2, 0));
    }
    private void setupCamera(UsbCamera camera){
        camera.setConnectVerbose(0); // so it doesn't spam the console with annoying messages if it's disconnected
        if(camera.isConnected()){
            final double ratio = camera.getVideoMode().height / (double) camera.getVideoMode().width;
            camera.setVideoMode(VIDEO_MODE.pixelFormat, VIDEO_MODE.width, (int) (VIDEO_MODE.width * ratio), VIDEO_MODE.fps);
            camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
        } else {
            System.err.println("One of the cameras isn't plugged in. Not initializing");
            camera.close();
        }
    }

    @Override
    protected void onUpdate() {
        super.onUpdate();
        final TaskSystem taskSystem = taskSystemSupplier.get();
        requireNonNull(taskSystem);
        final TaskSystem.Task newTask = taskSystem.getCurrentTask();
        if(newTask != lastTask){
            lastTask = newTask;
            if(newTask == TaskSystem.Task.CARGO){
                if(cargo.isValid()) {
                    videoSink.setSource(cargo);
                }
            } else if(newTask == TaskSystem.Task.HATCH){
                if(hatch.isValid()) {
                    videoSink.setSource(hatch);
                }
            } else {
                throw new UnsupportedOperationException("Unsupported task: " + newTask);
            }
        }
    }

    @Override
    protected void onEnd(boolean peacefullyEnded) {
        super.onEnd(peacefullyEnded);
        System.out.println("Even though when I created this method I never expected it to be called... It's being called now. Hopefully you have good intentions");
        hatch.close();
        cargo.close();
        videoSink.close();
    }
}
