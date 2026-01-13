import cv2
import numpy as np
import matplotlib.pyplot as plt

# Image folder path
image_folder = '/home/dlee/ros2_ws/src/sar_simulation/sar_projects/Hummingbird/Image/1m_s'

# List of image paths
image_paths = [
    f'{image_folder}/8.5.png',
    f'{image_folder}/8.png',
    f'{image_folder}/7.5.png',
    f'{image_folder}/7.png',
    f'{image_folder}/6.5.png',
    f'{image_folder}/6.png',
    f'{image_folder}/5.5.png',
    f'{image_folder}/5.png',
    f'{image_folder}/4.5.png',
    f'{image_folder}/4.png',
    f'{image_folder}/3.5.png',
    f'{image_folder}/3.png',
    f'{image_folder}/2.5.png',
    f'{image_folder}/2.png',
    f'{image_folder}/1.5.png',
    f'{image_folder}/1.png',
    f'{image_folder}/0.5.png',
]

# Load grayscale images
images = [cv2.imread(img_path, cv2.IMREAD_GRAYSCALE) for img_path in image_paths]

# Lucas-Kanade Optical Flow parameters
lk_params = dict(winSize=(5, 5),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.01))

# Good Features to Track parameters
feature_params = dict(maxCorners=50,
                      qualityLevel=0.2,
                      minDistance=2,
                      blockSize=3)

# Camera info (used for visualization only)
horizontal_fov = 1.435  # radians
image_width = 64
angle_per_pixel = horizontal_fov / image_width  # radians per pixel

# Time between frames
delta_t = 0.5  # seconds

for i in range(len(images) - 1):
    old_gray = images[i]
    frame_gray = images[i + 1]

    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

    if p0 is None or len(p0) == 0:
        print(f"No features found in frame {i}. Skipping.")
        continue

    # Optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

    good_new = p1[st == 1]
    good_old = p0[st == 1]

    if len(good_new) < 2:
        print(f"Too few tracked features in frame {i}. Skipping.")
        continue

    # 중심점 기준으로 feature spread 계산 (화면상 확산 정도)
    center_old = np.mean(good_old, axis=0)
    center_new = np.mean(good_new, axis=0)

    dist_old = np.linalg.norm(good_old - center_old, axis=1)
    dist_new = np.linalg.norm(good_new - center_new, axis=1)

    scale_old = np.mean(dist_old)
    scale_new = np.mean(dist_new)

    scale_change_rate = (scale_new - scale_old) / delta_t

    # TTC 계산 (Depth 없이 스케일 변화 기반)
    if scale_change_rate > 1e-5:
        ttc = scale_old / scale_change_rate
    else:
        ttc = float('inf')

    print(f"Frame {i} to {i+1}: Estimated TTC (scale-based, no depth) = {ttc:.2f} seconds")

    # Draw optical flow tracks
    mask = np.zeros_like(cv2.cvtColor(old_gray, cv2.COLOR_GRAY2BGR))
    for j, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
        frame = cv2.circle(cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR), (int(a), int(b)), 5, (0, 0, 255), -1)

    img = cv2.add(frame, mask)

    # Display
    plt.figure(figsize=(6, 6))
    plt.title(f"Optical Flow: Frame {i} to {i+1}\nTTC (no depth) = {ttc:.2f} sec")
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.axis('off')
    plt.show()
