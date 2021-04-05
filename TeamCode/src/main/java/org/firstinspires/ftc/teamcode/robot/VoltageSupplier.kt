package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.VoltageSupplier

class VoltageSupplier: MultithreadedRobotModule(), VoltageSupplier{
    private val defaultVoltage = 12.0
    override var voltage: Double = defaultVoltage
    private val pauseTime = 100.0
    private val voltageTime = ElapsedTime()

    lateinit var voltageSensor: VoltageSensor

    override fun initialize() {
        voltageSensor = WoENHardware.controlHubVoltageSensor
    }

    override fun updateControlHub() {

        if(voltageTime.milliseconds() > pauseTime) {
            voltage = (voltage + voltageSensor.voltage) / 2
            voltageTime.reset()
            //TODO Rolling average
            //TODO Query timeout
        }

    }
}