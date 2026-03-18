import numpy as np
from PIL import Image

def make_ceiling_sine_texture(
    size_m: float = 32.0,
    period_m: float = 0.25,     # L = 0.25 m
    px: int = 8192,             # 4096 or 8192 추천
    out_path: str = "ceiling_sine_L025_32m_8192.png",
    invert: bool = False,       # 필요하면 밝/어둠 반전
    gamma: float = 1.0,         # 1.0 유지 권장(인쇄용 아님)
    I0: int = 255
):
    """
    GZ Sim용 32m×32m 사인파 밝기 텍스처 생성.
    - 이 PNG를 SDF에서 <albedo_map>으로 쓰고
    - geometry size를 <size>32 32 ...</size>로 하면
      물리적으로 32m에 1:1 매핑됨.
    """

    # 좌표 (m): [0, size_m)
    x = np.linspace(0.0, size_m, px, endpoint=False, dtype=np.float32)
    y = np.linspace(0.0, size_m, px, endpoint=False, dtype=np.float32)

    # 논문 형태: Ix, Iy 만든 뒤 평균
    Ix = 0.5 * (np.sin(2.0 * np.pi * x / period_m) + 1.0)  # 0..1
    Iy = 0.5 * (np.sin(2.0 * np.pi * y / period_m) + 1.0)  # 0..1
    I = 0.5 * (Ix[None, :] + Iy[:, None])                  # 0..1

    if invert:
        I = 1.0 - I

    # 감마 (선택): 1.0이면 변화 없음
    if gamma != 1.0:
        I = np.clip(I, 0.0, 1.0) ** (1.0 / float(gamma))

    img_u8 = np.clip(np.round(I * I0), 0, 255).astype(np.uint8)
    Image.fromarray(img_u8, mode="L").save(out_path)

    periods = size_m / period_m
    px_per_period = px / periods

    print(f"Saved: {out_path}")
    print(f"Physical: {size_m}m x {size_m}m, Period L={period_m}m")
    print(f"Periods across one axis: {periods:.1f}  (should be 128 for 32m/0.25m)")
    print(f"Image: {px} x {px} px")
    print(f"Pixels per period: {px_per_period:.2f} px/period")
    print("Use in SDF with: <size>32 32 0.01</size> and <albedo_map> pointing to this PNG.")

if __name__ == "__main__":
    # ✅ 권장 기본값: 8192 (부드러움), 부담되면 4096로 낮추기
    make_ceiling_sine_texture(
        size_m=32.0,
        period_m=0.25,
        px=8192,
        out_path="ceiling_sine_L025_32m_8192.png",
        invert=False,
        gamma=1.0
    )
