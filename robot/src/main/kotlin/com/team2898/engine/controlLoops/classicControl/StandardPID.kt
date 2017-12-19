package com.team2898.engine.controlLoops

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.BoundaryException

/**
 * Standard PID control loop
 * Everything is synchronous, which means that you have to manually call the update()  function
 * Adapted with modifications from Team 254's library
 * @author Solomon
 * TODO {Replace importdoc with the actual things from IPID interface (then delete ipid)}
 */

open class StandardPID {
    var m_Kp: Double = 0.0
    var m_Ki: Double = 0.0
    var m_Kd: Double = 0.0
    var m_Kf: Double = 0.0

    var m_integrator: Double = 0.0
    var m_derivator: Double  = 0.0

    var m_integratorMax: Double = Double.POSITIVE_INFINITY

    var m_maxOut: Double = Double.POSITIVE_INFINITY
    var m_minOut: Double = Double.NEGATIVE_INFINITY

    var m_maxIn: Double = Double.POSITIVE_INFINITY
    var m_minIn: Double = Double.NEGATIVE_INFINITY

    var m_wraparound: Boolean = false

    var m_prevError: Double = 0.0

    var m_setpoint: Double  = 0.0
    var m_error: Double     = 0.0
    var m_output: Double    = 0.0

    var m_deadband: Double  = 0.0

    var m_input: Double     = 0.0

    var m_lastInput: Double = 0.0

    var m_lastTime: Double = 0.0

    var m_updated = false // this will prevent extraneous calling of ontarget and stuff

    constructor () {}

    /** Creates PID controller with PIDF coefficients given
     * @param Kp
     *  proportonal coefficient
     * @param Ki
     *  integral coefficient
     * @param Kd
     *  derivative coefficient
     * @param Kf
     *  feed-forward coefficient
     */
    constructor (Kp: Double = 0.0, Ki: Double = 0.0, Kd: Double = 0.0, Kf: Double = 0.0, integratorMax: Double = Double.POSITIVE_INFINITY, currentTime: Double = Timer.getFPGATimestamp()) {
        m_Kp = Kp
        m_Ki = Ki
        m_Kd = Kd
        m_Kf = Kf
        m_integratorMax = integratorMax
        m_lastTime = currentTime
    }

    /**
     *  {@inheritDoc}
     */
    fun update(inputVal: Double, currentTime: Double = Timer.getFPGATimestamp()): Double {
        var dt = currentTime - m_lastTime
        m_lastTime = currentTime

        m_lastInput = inputVal
        m_input = inputVal
        m_error = m_setpoint - m_input

        // Wraparound error if sensor is continuous
        if (m_wraparound && (Math.abs(m_error) > (m_maxIn - m_minIn) / 2)) {
            m_error = if (m_error > 0) m_error - m_maxIn + m_minIn else m_error + m_maxIn + m_minIn
        }

        // Update integral
        if (m_minOut < m_error * m_Kp && m_error * m_Kp < m_maxOut)
            m_integrator += m_error
        else m_integrator = 0.0

        // Update derivative
        m_derivator = (m_error - m_prevError) * dt

        // We want to keep m_error as it is, but we don't want to do proportional control if error is less than deadband
        val kpError: Double = if (Math.abs(m_error) < m_deadband) 0.0 else m_error

        m_output = m_Kp*kpError + m_Ki * m_integrator + m_Kd * m_derivator

        m_prevError = m_error

        // Clamp output
        m_output = if (m_output > m_maxOut) m_maxOut else if (m_output < m_minOut) m_minOut else m_output

        m_updated = true

        return m_output
    }

    /**
     * {@inheritDoc}
     */
    fun setPID(Kp: Double, Ki: Double, Kd: Double, Kf: Double) { // Defaults are set in IPID interface
        m_Kp = Kp
        m_Ki = Ki
        m_Kd = Kd
        m_Kf = Kf
    }


    /**
     * {@inheritDoc}
     */
    fun setContinuous(continuous: Boolean) { // Contunious default of true is set in IPID interface
        m_wraparound = continuous
    }

    /**
     * {@inheritDoc}
     */
    fun setDeadband(deadband: Double) {
        m_deadband = deadband
    }

    /**
     * {@inheritDoc}
     */
    fun setInputRange(minIn: Double, maxIn: Double) {
        if (minIn > maxIn) throw BoundaryException("Lower input bound is greater than upper bound")
        m_minIn = minIn
        m_maxIn = maxIn
        setSetpoint(m_setpoint) // So that the setpoint gets bounded to the new range
    }

    /**
     * {@inheritDoc}
     */
    fun setOutputRange(minOut: Double, maxOut: Double) {
        if (minOut > maxOut) throw BoundaryException("Lower output bound is greater than upper bound")

        m_minOut = minOut
        m_maxOut = maxOut
    }

    /**
     * {@inheritDoc}
     */

    fun setSetpoint(setpoint: Double) {
        m_setpoint = if (setpoint > m_maxIn) m_maxIn else if (setpoint < m_minIn) m_minIn else setpoint // Bound setpoint to min and max input values
    }

    /**
     * {@inheritDoc}
     */
    fun getError(): Double {
        return m_error
    }

    /**
     * {@inheritDoc}
     */
    fun onTarget(tolerance: Double): Boolean {
        return m_updated && Math.abs(m_lastInput - m_setpoint) < tolerance  // m_updated prevents immediate ontarget calls from being true
    }

    /**
     * {@inheritDoc}
     */
    fun reset() {
        m_prevError = 0.0
        m_lastInput = 0.0
        m_derivator = 0.0
        m_integrator = 0.0
        m_error = 0.0
        m_output = 0.0
        m_setpoint = 0.0
        m_updated = false
    }
}
