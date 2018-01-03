package com.team2898.robot.motion

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.comms.SerializableTypes.*
import com.team2898.engine.math.quaternionFromEuler
import com.team2898.robot.NTHandler
import kotlinx.serialization.json.JSON

object OdometryNTReporter {
    // need to send off a Twist and Pose
    @Synchronized
    fun updateNavMsgs() {
        val currentPose = RobotState.pose

        val r0 = RobotState[0]
        val r1 = RobotState[1]

        // Compute twist --- linear (delta position: vector3) and angular (delta rotation: vector3)
        val deltaPosition = SerializableVector3(
                x = r0.x - r1.x,
                y = r0.y - r1.y,
                z = 0.0
        )
        val deltaRotation = SerializableVector3(
                x = 0.0,
                y = 0.0,
                z = r0.theta - r1.theta
        )
        val twist = SerializableTwist(linear = deltaPosition, angular = deltaRotation)


        // Compute pose --- position (point) and orientation (quaternion)
        val position = SerializablePoint(
                x = r0.x, y = r0.y, z = 0.0
        )
        val q = quaternionFromEuler(pitch = 0.0, roll = 0.0, yaw = r0.theta)
        val orientation = SerializableQuaternion(x = q.q0, y = q.q1, z = q.q2, w = q.q3)
        val pose = SerializablePose(position = position, orientation = orientation)


        val serializedTwist = JSON.stringify(twist)
        val serializedPose = JSON.stringify(pose)

        NTHandler.navTable.putString("twist", serializedTwist)
        NTHandler.navTable.putString("pose", serializedPose)
    }

}
