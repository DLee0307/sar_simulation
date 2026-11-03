import numpy as np
from PIL import Image
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
import os  # 경로 처리를 위해 추가

def make_sine_pattern_image(size_mm=1000, period_mm=250, dpi=150, I0=255):
    """
    1m x 1m 사인파 그레이스케일 패턴 이미지(PIL Image)를 생성.
    """
    px_per_mm = dpi / 25.4
    W = int(round(size_mm * px_per_mm))
    H = int(round(size_mm * px_per_mm))

    x_mm = np.linspace(0, size_mm, W, endpoint=False, dtype=np.float32)
    y_mm = np.linspace(0, size_mm, H, endpoint=False, dtype=np.float32)

    Ix = (I0 / 2.0) * (np.sin(2.0 * np.pi * x_mm / period_mm) + 1.0)
    Iy = (I0 / 2.0) * (np.sin(2.0 * np.pi * y_mm / period_mm) + 1.0)

    I = (Ix[None, :] + Iy[:, None]) / 2.0

    img_u8 = np.clip(np.round(I), 0, 255).astype(np.uint8)
    img = Image.fromarray(img_u8, mode="L")

    return img, (W, H)

def save_pattern_pdf(
    pdf_path="ceiling_sine_L025_1m.pdf",
    size_mm=1000,
    period_mm=250,
    dpi=150,
):
    """
    1m x 1m 패턴을 코드 위치와 동일한 경로에 PDF로 저장.
    """
    # 1. 현재 이 스크립트 파일(.py)이 위치한 디렉토리 경로를 가져옵니다.
    # 만약 주피터 노트북 환경이라면 os.getcwd()를 사용하세요.
    try:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    except NameError:
        base_dir = os.getcwd()

    # 2. 파일명을 절대 경로와 결합합니다.
    full_path = os.path.join(base_dir, pdf_path)

    img, (W, H) = make_sine_pattern_image(size_mm=size_mm, period_mm=period_mm, dpi=dpi)

    page_w = size_mm * mm
    page_h = size_mm * mm

    # 수정된 full_path를 사용하여 캔버스 생성
    c = canvas.Canvas(full_path, pagesize=(page_w, page_h))

    img_reader = ImageReader(img)
    c.drawImage(img_reader, 0, 0, width=page_w, height=page_h, mask=None)

    c.showPage()
    c.save()

    print(f"Saved PDF to: {full_path}")
    print(f"- Page size: {size_mm}mm x {size_mm}mm")
    print(f"- Raster source: {W} x {H} px @ {dpi} dpi")
    print(f"- Period: {period_mm} mm")

if __name__ == "__main__":
    save_pattern_pdf(
        pdf_path="ceiling_sine_L025_1m_150dpi.pdf",
        size_mm=1000,
        period_mm=250,
        dpi=150
    )