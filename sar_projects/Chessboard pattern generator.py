import numpy as np
import matplotlib.pyplot as plt

# 파라미터 설정
I0 = 1.0  # Intensity 최대값
L_values = [0.33]  # L 값 리스트
x = np.linspace(-10, 10, 500)  # x 좌표
y = np.linspace(-10, 10, 500)  # y 좌표
X, Y = np.meshgrid(x, y)

# 체스보드 패턴 생성 및 저장
for L in L_values:
    Ix = (I0 / 2) * (np.sin(2 * np.pi * X / L) + 1)
    Iy = (I0 / 2) * (np.sin(2 * np.pi * Y / L) + 1)
    I = (Ix + Iy) / 2 
    
    # 이미지 플롯 생성
    plt.figure(figsize=(10, 10))
    plt.imshow(I, extent=(-10, 10, -10, 10), cmap='gray', origin='lower')
    plt.axis('off')
    plt.gca().set_axis_off()  # 축 박스도 제거
    plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)

    # 이미지 저장
    filename = f'chessboard_L_{L}.jpg'
    plt.savefig(filename, dpi=400)
    print(f"이미지 저장 완료: {filename}")
    plt.close()

print("모든 이미지 생성 완료.")
