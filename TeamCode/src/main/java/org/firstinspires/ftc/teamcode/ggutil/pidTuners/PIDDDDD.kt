package org.firstinspires.ftc.teamcode.ggutil.pidTuners

import CommonUtilities.PIDParams
import CommonUtilities.Result
import com.dacodingbeast.pidtuners.Algorithm.Dt
import com.dacodingbeast.pidtuners.Simulators.AngleRange
import com.dacodingbeast.pidtuners.Simulators.SlideRange
import com.dacodingbeast.pidtuners.Simulators.Target
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

class PIDFcontroller(var params: PIDParams, val isSimulator: Boolean = false) {

    private var prevError = 0.0
    private var integral = 0.0

    // Pre-calculated constants
    private val dtInverse = 1.0 / Dt
    private val hasFF = params.kf != 0.0

    private var minIntegral: Double = -1.0
    private var maxIntegral: Double = 1.0

    private lateinit var timer: ElapsedTime

    init {
        if (!isSimulator) timer = ElapsedTime()
    }

    private inline fun getLoopTime(): Double{
        return if (isSimulator){
            Dt
        }else{
            timer.seconds()
        }
    }

    private inline fun getLoopTimeInverse(): Double{
        return if (isSimulator){
            dtInverse
        }else{
            1/ (timer.seconds())
        }
    }

    fun calculate(position: com.dacodingbeast.pidtuners.Simulators.Target, obstacle: Target?): CommonUtilities.Result {

        when (position) {
            is AngleRange -> {
                val (_, error) = AngleRange.findDirectionAndError(position, obstacle as AngleRange?)

                val ff = if (hasFF) {
                    val sinVal = sin(position.start)
                    if (position.start > 0.0) max(0.0, sinVal) * params.kf
                    else min(0.0, sinVal) * params.kf
                } else 0.0

                return calculateControl(error, ff)
            }

            is SlideRange -> {
                val error = position.stop - position.start
                return calculateControl(error, 0.0)
            }
        }
    }

    private inline fun calculateControl(error: Double, ff: Double): CommonUtilities.Result {
        integral += error * getLoopTime()
        integral = integral.coerceIn(minIntegral,maxIntegral)

        val derivative = (error - prevError) * getLoopTimeInverse()
        prevError = error

        val controlEffort = (error * params.kp + integral * params.ki + derivative * params.kd + ff)
            .coerceIn(-1.0, 1.0)

        if(!isSimulator) timer.reset()

        return Result(controlEffort, error)
    }

    fun reset() {
        prevError = 0.0
        integral = 0.0
    }
}