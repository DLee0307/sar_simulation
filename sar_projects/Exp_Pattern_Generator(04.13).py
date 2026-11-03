import numpy as np
from PIL import Image
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader


def make_sine_pattern_image(size_mm=1500, period_mm=200, dpi=150, I0=255):
    """
    1.5m x 1.5m 사인파 그레이스케일 패턴 생성
    - size_mm: 패턴 전체 크기 (mm)
    - period_mm: 공간 주기 (mm)  (0.20 m = 200 mm)
    - dpi: 해상도
    """

    px_per_mm = dpi / 25.4
    W = int(round(size_mm * px_per_mm))
    H = int(round(size_mm * px_per_mm))

    x_mm = np.linspace(0, size_mm, W, endpoint=False, dtype=np.float32)
    y_mm = np.linspace(0, size_mm, H, endpoint=False, dtype=np.float32)

    # 논문 형태 (x, y 각각 사인 → 평균)
    Ix = (I0 / 2.0) * (np.sin(2.0 * np.pi * x_mm / period_mm) + 1.0)
    Iy = (I0 / 2.0) * (np.sin(2.0 * np.pi * y_mm / period_mm) + 1.0)

    I = (Ix[None, :] + Iy[:, None]) / 2.0

    img_u8 = np.clip(np.round(I), 0, 255).astype(np.uint8)
    img = Image.fromarray(img_u8, mode="L")

    return img, (W, H)


def save_pattern_pdf(
    pdf_path="ceiling_sine_L020_1p5m.pdf",
    size_mm=1500,
    period_mm=200,
    dpi=150,
):
    """
    1500mm x 1500mm 패턴을 정확한 실측 크기의 PDF로 저장
    """

    img, (W, H) = make_sine_pattern_image(
        size_mm=size_mm,
        period_mm=period_mm,
        dpi=dpi
    )

    # PDF 페이지 크기 (mm → point 자동 변환)
    page_w = size_mm * mm
    page_h = size_mm * mm

    c = canvas.Canvas(pdf_path, pagesize=(page_w, page_h))

    img_reader = ImageReader(img)

    # 페이지 전체에 정확히 맞게 삽입
    c.drawImage(
        img_reader,
        0, 0,
        width=page_w,
        height=page_h,
        mask=None
    )

    c.showPage()
    c.save()

    print(f"Saved PDF: {pdf_path}")
    print(f"- Page size: {size_mm} mm x {size_mm} mm")
    print(f"- Resolution: {W} x {H} px @ {dpi} dpi")
    print(f"- Period: {period_mm} mm")
    print(f"- Periods across: {size_mm/period_mm:.2f}")


if __name__ == "__main__":
    save_pattern_pdf(
        pdf_path="ceiling_sine_L020_1p5m_150dpi.pdf",
        size_mm=1500,
        period_mm=200,
        dpi=150
    )