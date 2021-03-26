package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.VoltageSupplier

class VoltageSupplier: MultithreadedRobotModule(), VoltageSupplier{
    private val defaultVoltage = 12.0
    override var voltage: Double = defaultVoltage

    lateinit var voltageSensor: VoltageSensor

    override fun initialize() {
        voltageSensor = WoENHardware.controlHubVoltageSensor
    }

    override fun updateControlHub() {
        /*
        if() ...
        TODO Rolling average
        TODO Query timeout
         */
        voltage = voltageSensor.voltage
    }
}