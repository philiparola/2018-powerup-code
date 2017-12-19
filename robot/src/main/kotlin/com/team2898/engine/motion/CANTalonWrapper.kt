package com.team2898.engine.motion

import com.ctre.CANTalon
import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.signalProcessing.MovingAverageFilter
import com.team2898.engine.types.MinMax
import com.team2898.engine.util.clamp
import edu.wpi.first.wpilibj.Timer

/** Custom CANTalon wrapper enabling databinding and easy motion profiles. Also can automatically optimize status frames.
 * @param deviceID CAN ID of the device
 * @param stateMachineHz Hz to run the internal state machine at. 10 is good for low priority subsystems, 100 for high
 * @param bufferPushHz Formality, 50 is good
 * @param generalFrameHz Minimum and maximum
 * @param framerateFilterSize The number of previous unique setpoint delays to average
 */
class CANTalonWrapper(
        deviceID: Int,
        stateMachineHz: Double = 10.0,
        bufferPushHz: Double = 50.0,
        generalFrameHz: MinMax = MinMax(10.0, 100.0, 10.0),
        useAdaptiveFramerate: Boolean = true,
        framerateFilterSize: Int = 100
) : CANTalon(deviceID) {

    companion object {
        fun createSlave(deviceID: Int, other: CANTalon): CANTalonWrapper {
            return CANTalonWrapper(deviceID, stateMachineHz = 10.0, bufferPushHz = 50.0).apply {
                setSlave(other)
            }
        }
    }

    init {
        setStatusFrameRateMs(StatusFrameRate.General, (1000.0/generalFrameHz.norm).toInt())
    }

    val framerateFilter = MovingAverageFilter(framerateFilterSize).apply {
        // setLinearWeighting()
        setEqualWeighting()
        addValue(1 / generalFrameHz.norm)
    }

    var lastSetpointTimestamp = Double.NaN
    val useAdaptiveFramerate: Boolean by lazy { useAdaptiveFramerate } // Quick hack so we can use this from set()

    var lastSetpoint: Double = Double.NaN
    var lastControlMode: TalonControlMode = controlMode ?: TalonControlMode.PercentVbus

    val profileHelper = CANTalonProfileHelper(this, bufferPushHz)

    var lastPIDProfile: Int = -1

    var stateMachineLooper: AsyncLooper = AsyncLooper(stateMachineHz) {
        // If we're using an adaptive framerate for the general status frames, set it to the average of how often our
        // setpoints change
        if (useAdaptiveFramerate) {
            var framerate = clamp(framerateFilter.getAverage(), 1 / generalFrameHz.min, 1 / generalFrameHz.max)
            setStatusFrameRateMs(StatusFrameRate.General, (framerate * 1000).toInt())
        }
    }

    override fun set(value: Double) {
        if (lastSetpoint != value) {
            super.set(value)
            lastSetpoint = value

            // Update moving average filter if we're doing the adaptive framerate thingy
            if (useAdaptiveFramerate) {
                if (lastSetpointTimestamp == Double.NaN) lastSetpointTimestamp = Timer.getFPGATimestamp()
                framerateFilter.addValue(Timer.getFPGATimestamp() - lastSetpointTimestamp)
                lastSetpointTimestamp = Timer.getFPGATimestamp()
            }
        }
    }

    fun setOpen(value: Double) {
        setOpenLoop()
        set(value)
    }

    fun setVel(value: Double) {
        setVelocityControl()
        set(value)
    }

    fun setPos(value: Double) {
        setPositionControl()
        set(value)
    }

    override fun changeControlMode(controlMode: TalonControlMode) {
        if (lastControlMode.value != controlMode.value) {
            super.changeControlMode(controlMode)
            lastControlMode = controlMode
        }
    }

    override fun setProfile(profile: Int) {
        if (lastPIDProfile != profile) {
            lastPIDProfile = profile
            super.setProfile(profile)
        }
    }

    fun setOpenLoop() = changeControlMode(TalonControlMode.PercentVbus)
    fun setPositionControl() = changeControlMode(TalonControlMode.Position)
    fun setVelocityControl() = changeControlMode(TalonControlMode.Speed)

    fun setMagEncoder() {
        setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative)
        configEncoderCodesPerRev(4096)
    }

    fun setSlave(master: CANTalon): CANTalonWrapper {
        changeControlMode(TalonControlMode.Follower)
        super.set(master.deviceID.toDouble())
        return this
    }

    infix fun slaveTo(master: CANTalon): CANTalonWrapper {
        return setSlave(master)
    }

}
