package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.math.sign

class RegulatorPIDVAS (var motorRegulator: DcMotorEx, var voltageSensor: VoltageSensor, private val kP: Double = 0.0, private val kD: Double = 0.0, private val kI: Double = 0.0, private val kV: Double = 0.0, private val kA: Double = 0.0, private val kS: Double = 0.0, private val maxI: Double = 600000.0, private val kV_referenceVoltage: Double = 12.485){
    private val updateTime = ElapsedTime()
    private var velocityError = 0.0
    private var velocityErrorOld = 0.0
    private var D = 0.0
    private var P = 0.0
    private var I = 0.0
    private var V = 0.0
    private var A = 0.0
    private var S = 0.0
    private var power = 0.0
    private var timeOld = 0.0
    private var timeDelta = 0.0
    private var voltageDelta = 0.0
    private var velocityTargetOld = 0.0
    private var currentVelocity = 0.0
    private val maxInt16 = 32767.0
    fun updateRegulator(target: Double):Double {
        timeDelta = updateTime.seconds() - timeOld
        timeOld = updateTime.seconds()
        if (target != 0.0) {
            currentVelocity = motorRegulator.velocity
            voltageDelta = kV_referenceVoltage / voltageSensor.voltage
            velocityError = target - currentVelocity
            P = velocityError * kP
            D = (velocityError - velocityErrorOld) * kD / timeDelta
            I += (kI * velocityError) * timeDelta
            if (abs(I) > maxI) I = sign(I) * maxI
            V = kV * target * voltageDelta
            A = kA * (target - velocityTargetOld) / timeDelta * voltageDelta
            S = kS * sign(target) * voltageDelta
            power = (P + I + D + V + A + S) / maxInt16
            velocityErrorOld = velocityError
            velocityTargetOld = target
        } else {
            currentVelocity = 0.0
            voltageDelta = 0.0
            velocityError = 0.0
            power = 0.0
            velocityErrorOld = 0.0
            I = 0.0
            velocityTargetOld = 0.0
        }
        return power
    }
}