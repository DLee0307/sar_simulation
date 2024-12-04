import cv2
import numpy as np
import glob

# 체스보드 코너의 행(가로 코너 수)과 열(세로 코너 수)
chessboard_size = (9, 6)  # 9x6 체스보드
square_size = 0.1  # 체스보드 한 칸의 크기 (단위: meter, 예: 2.5cm)

# 3D 객체 좌표 준비 (체스보드 코너의 실제 좌표)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # 실제 크기 적용

# 3D 좌표와 2D 좌표 저장
objpoints = []  # 3D 점 (체스보드의 실제 좌표)
imgpoints = []  # 2D 점 (이미지 내 코너 좌표)

# 체스보드 이미지 경로 설정
images = glob.glob('/home/dlee/ros2_ws/src/sar_simulation/sar_projects/123/*.png')  # 이미지 경로

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 체스보드 코너 찾기
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # 코너 시각화
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        #cv2.imshow('Corners', img)
        #cv2.waitKey(500)

#cv2.destroyAllWindows()

# 카메라 칼리브레이션
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# 초점 거리 추출
fx = camera_matrix[0, 0]  # x축 초점 거리
fy = camera_matrix[1, 1]  # y축 초점 거리

print("Camera Matrix:")
print(camera_matrix)
print(f"Focal Length (fx, fy): ({fx:.2f}, {fy:.2f})")
