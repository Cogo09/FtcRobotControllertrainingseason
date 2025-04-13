package org.firstinspires.ftc.teamcode.ggutil.vv

import org.gentrifiedApps.gentrifiedAppsUtil.velocityVision.classes.CameraParams
import org.gentrifiedApps.gentrifiedAppsUtil.velocityVision.classes.LensIntrinsics
import org.opencv.calib3d.Calib3d
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

class HomographicMatrix {
    fun process(input: Mat, cameraParams: CameraParams): Mat {
        val srcPoints = MatOfPoint2f(
            Point(10.0, 10.0),
            Point(100.0, 10.0),
            Point(100.0, 100.0),
            Point(10.0, 100.0)
        )

        // Define destination points (e.g., where you want to map the source points)
        val dstPoints = MatOfPoint2f(
            Point(50.0, 50.0),
            Point(200.0, 50.0),
            Point(200.0, 200.0),
            Point(50.0, 200.0)
        )

        // Define camera intrinsic parameters (fx, fy, cx, cy)
        val cameraMatrix = Mat(3, 3, CvType.CV_32FC1)
        cameraMatrix.put(0, 0, cameraParams.lensIntrinsics.fx!!.toDouble()) // focal length x
        cameraMatrix.put(1, 1, cameraParams.lensIntrinsics.fy!!.toDouble()) // focal length y
        cameraMatrix.put(0, 2, cameraParams.lensIntrinsics.cx!!.toDouble()) // principal point x
        cameraMatrix.put(1, 2, cameraParams.lensIntrinsics.cy!!.toDouble()) // principal point y

        // Define camera extrinsic parameters (rotation and translation)
        val rotVec = Mat(3, 1, CvType.CV_32FC1)
        val transVec = Mat(3, 1, CvType.CV_32FC1)

        // Use findHomography with the camera matrix
        val homographyMatrix = Calib3d.findHomography(srcPoints, dstPoints, Calib3d.RANSAC, 3.0)
        // Apply perspective warp using the homography matrix
        val warpedImage = Mat()
        Imgproc.warpPerspective(input, warpedImage, homographyMatrix, input.size())
        return warpedImage
    }
    fun warpToTopDown(input: Mat, cameraParams: CameraParams): Mat {
        // Extract intrinsics
        val fx = cameraParams.lensIntrinsics.fx!!
        val fy = cameraParams.lensIntrinsics.fy!!
        val cx = cameraParams.lensIntrinsics.cx!!
        val cy = cameraParams.lensIntrinsics.cy!!

        // Yaw, pitch, roll in radians
        val yaw = Math.toRadians(cameraParams.rotationalVector.yaw)
        val pitch = Math.toRadians(cameraParams.rotationalVector.pitch)
        val roll = Math.toRadians(cameraParams.rotationalVector.roll)

        // Translation vector
        val tx = cameraParams.translationalVector.x!!
        val ty = cameraParams.translationalVector.y!!
        val tz = cameraParams.translationalVector.z!!

        // Define the rotation matrix
        val rotation = MatOfDouble(
            Math.cos(yaw), -Math.sin(yaw), 0.0,
            Math.sin(yaw), Math.cos(yaw), 0.0,
            0.0, 0.0, 1.0
        )

        // Define the transform matrix including rotation and translation
        val transform = MatOfDouble(
            fx, 0.0, cx + tx * (cx / fx - 1),
            0.0, fy, cy + ty * (cy / fy - 1),
            0.0, 0.0, 1.0
        )

        // Initialize the homography matrix
        val H = Mat.zeros(3, 3, CvType.CV_64F)

//        // Ensure the matrices have the correct dimensions
//        if (rotation.rows != 3 || rotation.cols != 3) {
//            throw IllegalArgumentException("Rotation matrix must be 3x3")
//        }
//        if (transform.rows != 3 || transform.cols != 3) {
//            throw IllegalArgumentException("Transform matrix must be 3x3")
//        }

        // Debug output to verify matrices are not null
        println("Rotation Matrix: $rotation")
        println("Transform Matrix: $transform")

        // Perform manual matrix multiplication for H = rotation * transform
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                H.put(i, j,
                    rotation.get(0, i)[0] * transform.get(j, 0)[0] +
                            rotation.get(1, i)[0] * transform.get(j, 1)[0] +
                            rotation.get(2, i)[0] * transform.get(j, 2)[0]
                )
            }
        }
        // Debug output to verify homography matrix is not null
        println("Homography Matrix: $H")

        // Warp the input image using the homography matrix
        val result = Mat()
        Imgproc.warpPerspective(input, result, H, Size(input.cols().toDouble(),
            input.rows().toDouble()
        ))

        return result
    }
}