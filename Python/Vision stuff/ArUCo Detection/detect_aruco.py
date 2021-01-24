import cv2
import numpy as np

#Load the dictionary that was used to generate the markers.
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
parameters =  cv2.aruco.DetectorParameters_create()

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Unable to capture video")
        break

    # Detect the markers in the image
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    # print(markerCorners)
    print(rejectedCandidates)
    print(len(rejectedCandidates))
    if len(rejectedCandidates) > 0:
        for i in range(len(rejectedCandidates)):
            start = (rejectedCandidates[i][0][0][0],rejectedCandidates[i][0][0][1])
            end = (rejectedCandidates[i][0][3][0],rejectedCandidates[i][0][3][1])
            cv2.rectangle(frame, start, end, (58, 23, 187), 3)
    cv2.imshow("aruco detection",  frame)

    if len(markerIds) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs)
        # draw axis for each marker
        for i in range(0, len(markerIds)):
            cv2.aruco.drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

#
#
#

cap = cv2.VideoCapture(0)

cv::Mat cameraMatrix, distCoeffs
readCameraParameters(cameraMatrix, distCoeffs)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

while True:
    ret, frame = cap.read()
    frame.copyTo(imageCopy)
    cv::aruco::detectMarkers(image, dictionary, corners, ids)
    # if at least one marker detected
    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids)
        std::vector<cv::Vec3d> rvecs, tvecs
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs)
        # draw axis for each marker
        for(int i=0 i<ids.size() i++)
            cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1)
    }
    cv::imshow("out", imageCopy)
    char key = (char) cv::waitKey(waitTime)
    if (key == 27)
        break
}