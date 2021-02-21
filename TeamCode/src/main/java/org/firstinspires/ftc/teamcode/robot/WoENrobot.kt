package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.WoENHardware.assignHardware
import org.firstinspires.ftc.teamcode.robot.WoENHardware.lynxModules
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.openftc.revextensions2.ExpansionHubEx
import java.util.*

object WoENrobot {
    var wobbleManipulator = ServoWobbleManipulator()
    var openCVNode = OpenCVNodeWebcam()
    var conveyor = Conveyor2()
    var shooter = rpm()
    var telemetryDebugging = TelemetryDebugging()
    var ai = AI()

    //public static FakeRobot odometry = new FakeRobot();
    //public static FakeRobot drivetrain = odometry;
    var odometry = ThreeWheelOdometry()
    var drivetrain = MecanumDrivetrain()
    var movement = Movement(odometry, drivetrain)
    lateinit var opMode: LinearOpMode
    var robotIsInitialized = false
    val runTime = ElapsedTime()
    private val activeRobotModules = arrayOf(odometry, movement, drivetrain, wobbleManipulator, conveyor, shooter, telemetryDebugging, ai) //

    @Volatile
    var controlHubSpinCompleted = false

    @Volatile
    var expansionHubSpinCompleted = false

    @Volatile
    var spinCompleted = false
    lateinit var controlHub: ExpansionHubEx
    lateinit var expansionHub: ExpansionHubEx
    private lateinit var allHubs: List<LynxModule>
    var updateControlHub = Runnable {
        controlHub.standardModule.bulkCachingMode = BulkCachingMode.MANUAL
        while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
            controlHub.standardModule.clearBulkCache()
            Arrays.stream(activeRobotModules).forEach { obj: MultithreadRobotModule -> obj.updateControlHub() }
            controlHubSpinCompleted = true
        }
    }
    private var controlHubUpdater = Thread(updateControlHub)
    var updateExpansionHub = Runnable {
        expansionHub.standardModule.bulkCachingMode = BulkCachingMode.MANUAL
        while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
            expansionHub.standardModule.clearBulkCache()
            Arrays.stream(activeRobotModules).forEach { obj: MultithreadRobotModule -> obj.updateExpansionHub() }
            expansionHubSpinCompleted = true
        }
    }
    private var expansionHubUpdater = Thread(updateExpansionHub)
    var updateOther = Runnable {
        while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
            while ((!controlHubSpinCompleted || !expansionHubSpinCompleted) && opMode.opModeIsActive()) {
                Thread.yield()
            }
            Arrays.stream(activeRobotModules).forEach { obj: MultithreadRobotModule -> obj.updateOther() }
            controlHubSpinCompleted = false
            expansionHubSpinCompleted = false
            spinCompleted = true
        }
    }
    private var otherUpdater = Thread(updateOther)
    var updateRegulators = Runnable {
        setBulkCachingMode(BulkCachingMode.MANUAL)
        while (opMode.opModeIsActive() && !Thread.currentThread().isInterrupted) {
            clearBulkCaches()
            Arrays.stream(activeRobotModules).forEach { obj: MultithreadRobotModule -> obj.update() }
            spinCompleted = true
        }
    }
    private var regulatorUpdater = Thread(updateRegulators)
    fun FullInitWithCV(OpMode: LinearOpMode) {
        openCVNode.initialize(OpMode)
        forceInitRobot(OpMode)
    }

    @Synchronized
    private fun spinHubs() {
        controlHubSpinCompleted = false
        expansionHubSpinCompleted = false
        while ((!controlHubSpinCompleted || !expansionHubSpinCompleted) && opMode.opModeIsActive()) {
            Thread.yield()
        }
    }

    fun spinOnce() {
        spinCompleted = false
        while (!spinCompleted && opMode.opModeIsActive()) {
            Thread.yield()
        }
    }

    fun delay(delay_ms: Double) {
        val timer = ElapsedTime()
        timer.reset()
        while (timer.milliseconds() < delay_ms && opMode.opModeIsActive()) {
            Thread.yield()
        }
    }

    fun startRobot() {
        setLedColors(255, 166, 0)
        opMode.waitForStart()
        if (opMode.isStopRequested) return
        runTime.reset()
        Arrays.stream(activeRobotModules).forEach { obj: MultithreadRobotModule -> obj.start() }
        // regulatorUpdater.start();
        controlHubUpdater.start()
        expansionHubUpdater.start()
        otherUpdater.start()
        setLedColors(0, 237, 255)
        opMode.telemetry.addData("Status", "Running")
        opMode.telemetry.update()
    }

    fun initRobot(OpMode: LinearOpMode) {
        if (!robotIsInitialized) {
            forceInitRobot(OpMode)
            opMode.telemetry.addData("Status", "Initialization successful")
            opMode.telemetry.update()
        } else {
            opMode = OpMode
            Arrays.stream(activeRobotModules).forEach { robotModule: MultithreadRobotModule ->
                robotModule.setOpMode(
                    opMode
                )
            }
            // regulatorUpdater.interrupt();
            //  regulatorUpdater = new Thread(updateRegulators);
            opMode.telemetry.addData("Status", "Already initialized, ready")
            opMode.telemetry.update()
        }
    }

    fun forceInitRobot(OpMode: LinearOpMode) {
        opMode = OpMode
        opMode.telemetry.addData("Status", "Initializing...")
        opMode.telemetry.update()
        assignHardware(opMode.hardwareMap)
        controlHub = WoENHardware.controlHub
        expansionHub = WoENHardware.expansionHub
        allHubs = lynxModules
        setBulkCachingMode(BulkCachingMode.MANUAL)
        Arrays.stream(activeRobotModules).forEach { robotModule: MultithreadRobotModule ->
            robotModule.initialize(
                opMode
            )
        }
        if (regulatorUpdater.state != Thread.State.NEW) {
            regulatorUpdater.interrupt()
            regulatorUpdater = Thread(updateRegulators)
        }
        controlHubUpdater.interrupt()
        controlHubUpdater = Thread(updateControlHub)
        expansionHubUpdater.interrupt()
        expansionHubUpdater = Thread(updateExpansionHub)
        otherUpdater.interrupt()
        otherUpdater = Thread(updateOther)
        //stopAllMotors();
        robotIsInitialized = true
        opMode.telemetry.addData("Status", "Force initialized")
        opMode.telemetry.update()
    }

    fun simpleInit(OpMode: LinearOpMode) {
        initRobot(OpMode)
        startRobot()
    }

    fun setLedColors(r: Int, g: Int, b: Int) {
        controlHub.setLedColor(r, g, b)
        expansionHub.setLedColor(r, g, b)
    }

    fun setBulkCachingMode(mode: BulkCachingMode?) {
        for (module in allHubs) module.bulkCachingMode = mode
    }

    fun clearBulkCaches() {
        for (module in allHubs) module.clearBulkCache()
    }

    fun FullInit(OpMode: LinearOpMode) {
        forceInitRobot(OpMode)
    }
}