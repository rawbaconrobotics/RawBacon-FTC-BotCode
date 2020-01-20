package org.firstinspires.ftc.teamcode.BigDipper.RobotComponents;


/**
 * @author Raw Bacon Coders
 * Establishes Big Dipper motor accelerations processes
 */
public class DcMotorAccelerationThread implements Runnable {

    private static final int MAX_MOTORS = 8;
    private static final int UPDATES_PER_SECOND = 100;
    public static final int UPDATE_PERIOD_MS = 1000 / UPDATES_PER_SECOND;

    private DcMotorAccelerated[] acceleratedMotors = new DcMotorAccelerated[MAX_MOTORS];
    private boolean isAccelerationControlRunning = false;
    private int motorCount = 0;
    private Thread accelerationControlThread;

    public DcMotorAccelerationThread() {

    }

    /** Adds a motor */
    public void addMotor(DcMotorAccelerated aMotor) {
        acceleratedMotors[motorCount++] = aMotor;
    }

    /** Runs the proccess */
    @Override
    public synchronized void run() {
        while (isAccelerationControlRunning) {
            for (int i = 0; i < motorCount; i++) {
                acceleratedMotors[i].update();
            }

            try {
                Thread.sleep(UPDATE_PERIOD_MS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    /** Starts the proccess */
    public synchronized void start() {
        if(!isAccelerationControlRunning) {
            isAccelerationControlRunning = true;
            accelerationControlThread = new Thread(this);
            accelerationControlThread.start();
        }
    }

    /** Stops the proccess */
    public void stop() {
        isAccelerationControlRunning = false;

        for (int i = 0; i < motorCount; i++) {
            acceleratedMotors[i].stopMotorHard();
        }
    }

}
