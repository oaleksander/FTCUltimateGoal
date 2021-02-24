package org.firstinspires.ftc.teamcode.superclasses

interface Conveyor {
    fun enableConveyor(isEnabled: Boolean)
    fun setForceReverse(forceReverse: Boolean)
    fun setReverseAfterStop(doReverseOnStop: Boolean)
    fun setAutomaticConveyorStopping(doAutomaticConveyorStopping: Boolean)
}