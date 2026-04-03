import cv2
import numpy as np

url = "http://192.168.1.75:4747/video" 


cap = cv2.VideoCapture(url)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for corner, marker_id in zip(corners, ids):
            c = corner.reshape(4, 2).astype(int)
            top_left = tuple(c[0])

            cv2.putText(frame, f"id: {int(marker_id[0])}",
                        top_left, cv2.FONT_HERSHEY_PLAIN,
                        1.3, (255, 0, 255), 2)

    cv2.imshow("cam", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()