import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.animation import FFMpegWriter, PillowWriter
from matplotlib.patches import Rectangle
import os

# ============================================================
# Global settings
# ============================================================
I_0 = 255.0  # grayscale max

# Camera / sensor parameters
FoV = 82.22
FoV_rad = np.radians(FoV)

WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160

w = 3.6e-6     # pixel pitch [m]
f = 0.33e-3    # focal length [m]

O_up = WIDTH_PIXELS / 2.0
O_vp = HEIGHT_PIXELS / 2.0


def sinc(x):
    """safe sin(x)/x"""
    x = np.asarray(x, dtype=np.float64)
    out = np.ones_like(x)
    m = np.abs(x) > 1e-12
    out[m] = np.sin(x[m]) / x[m]
    return out


class ApproachPatternSim:
    """
    2D sine grayscale pattern on a plane + drone camera approaching vertically.
    Generates the camera view (pattern zooming in) and saves video.
    """

    def __init__(self, Lx=0.25, Ly=0.25, FPS=100):
        """
        Args:
            Lx, Ly: sine spatial periods on the plane [m]
            FPS: output video frame rate
        """
        self.Lx = float(Lx)
        self.Ly = float(Ly)
        self.FPS = float(FPS)

        # Precompute sensor-plane metric coordinates (u,v) for each pixel center
        up = np.arange(WIDTH_PIXELS, dtype=np.float64)
        vp = np.arange(HEIGHT_PIXELS, dtype=np.float64)
        U_p, V_p = np.meshgrid(up, vp)

        # pixel center at (u_p+0.5, v_p+0.5)
        self.u = (U_p + 0.5 - O_up) * w
        self.v = (V_p + 0.5 - O_vp) * w

    # ------------------------------------------------------------
    # World-plane pattern preview (optional)
    # ------------------------------------------------------------
    def preview_world_pattern(self, Surf_width=4, Surf_height=4, pixel_density=200,
                              X_cam=0.0, Y_cam=0.0, D_cam=1.0):
        """
        Visualize the world pattern + camera FoV rectangle at distance D_cam.
        (This is NOT the camera image; it's a plane view for sanity check.)
        """
        x = np.linspace(-0.5 * Surf_width, 0.5 * Surf_width, int(pixel_density * Surf_width))
        y = np.linspace(-0.5 * Surf_height, 0.5 * Surf_height, int(pixel_density * Surf_height))
        X, Y = np.meshgrid(x, y)

        P = np.sin(2*np.pi * X / self.Lx) + np.sin(2*np.pi * Y / self.Ly)
        I = I_0 * (0.5 + 0.25 * P)
        I = np.clip(I, 0, 255).astype(np.float32)

        Img_W = 2 * np.tan(FoV_rad/2) * D_cam
        Img_H = 2 * np.tan(FoV_rad/2) * D_cam

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.imshow(I, vmin=0, vmax=255, cmap=cm.gray, origin='upper',
                  extent=[x.min(), x.max(), y.min(), y.max()],
                  interpolation='none')
        ax.add_patch(Rectangle((X_cam-Img_W/2, Y_cam-Img_H/2), Img_W, Img_H,
                               fill=False, lw=1, color="tab:blue"))
        ax.set_title("World Plane 2D Sine Pattern + Camera FoV")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        plt.show()

    # ------------------------------------------------------------
    # Camera image synthesis
    # ------------------------------------------------------------
    def camera_image(self, D_cam, X_cam=0.0, Y_cam=0.0):
        """
        Pinhole projection from plane pattern to sensor image.
        As D decreases -> pattern zooms in.
        """
        D = float(D_cam)
        if D < 1e-6:
            D = 1e-6

        # Map sensor-plane coordinate -> world-plane coordinate
        # Xw = X_cam + (D/f)*u,  Yw = Y_cam + (D/f)*v
        Xw = X_cam + (D / f) * self.u
        Yw = Y_cam + (D / f) * self.v

        # Pixel-averaging (contrast roll-off) factor per direction
        # Original form: (f*L/(pi*D*w))*sin(pi*D*w/(L*f)) = sin(x)/x = sinc(x)
        ax = sinc(np.pi * D * w / (self.Lx * f))
        ay = sinc(np.pi * D * w / (self.Ly * f))

        P = ax * np.sin(2*np.pi * Xw / self.Lx) + ay * np.sin(2*np.pi * Yw / self.Ly)

        I = I_0 * (0.5 + 0.25 * P)  # keep in [0..255] (ideally)
        return np.clip(I, 0, 255).astype(np.float32)

    # ------------------------------------------------------------
    # Main: animate approach and save video
    # ------------------------------------------------------------
    def save_approach_video(self,
                            D0=1.0,
                            Vz=0.9,               # 접근 속도 (수직)
                            X0=0.0, Vx=0.0,        # 수평 이동 원하면 Vx != 0
                            Y0=0.0, Vy=0.0,        # 수평 이동 원하면 Vy != 0
                            D_min=0.05,            # 너무 가까워지면 stop
                            out_path="approach_zoom.mp4",
                            dpi=150,
                            bitrate=5000,
                            show_preview=False):
        """
        Save a video of the drone approaching the plane (pattern zooming in).

        Args:
            D0: initial distance [m]
            Vz: approach speed toward plane [m/s] (Vz>0 => D decreases)
            X0,Y0: initial lateral offsets on plane [m]
            Vx,Vy: lateral velocities [m/s]
            D_min: stop distance [m]
            out_path: output file path (.mp4 or .gif)
            show_preview: True => show window while also saving (optional)
        """

        if Vz <= 0:
            raise ValueError("Vz must be > 0 for approaching (D = D0 - Vz*t).")

        # total duration until reaching D_min
        T = (D0 - D_min) / Vz
        if T <= 0:
            raise ValueError("D0 must be > D_min.")

        n_frames = int(np.ceil(T * self.FPS))

        # Create figure for sensor view
        fig = plt.figure()
        ax = fig.add_subplot(111)

        I0 = self.camera_image(D0, X0, Y0)
        im = ax.imshow(I0, vmin=0, vmax=255, cmap=cm.gray, origin='upper', interpolation='none')

        title = ax.set_title("")
        ax.set_xlabel("u_p [pixels]")
        ax.set_ylabel("v_p [pixels]")
        fig.tight_layout()

        # Animation update
        def update(i):
            t = i / self.FPS
            D = D0 - Vz * t
            if D < D_min:
                D = D_min

            X = X0 + Vx * t
            Y = Y0 + Vy * t

            I = self.camera_image(D, X, Y)
            im.set_array(I)
            title.set_text(f"Approach view | t={t:.3f}s  D={D:.3f}m  Vz={Vz:.2f}m/s")
            return (im, title)

        anim = animation.FuncAnimation(
            fig, update,
            frames=n_frames,
            interval=1000.0 / self.FPS,
            blit=False,
            repeat=False
        )

        # Decide writer by extension
        ext = os.path.splitext(out_path)[1].lower()

        if ext == ".mp4":
            # Requires ffmpeg
            try:
                writer = FFMpegWriter(fps=self.FPS, bitrate=bitrate)
                anim.save(out_path, writer=writer, dpi=dpi)
                print(f"[SAVED] {out_path}")
            except Exception as e:
                print("[ERROR] MP4 save failed. Do you have ffmpeg installed?")
                print("Ubuntu: sudo apt-get install -y ffmpeg")
                raise e

        elif ext == ".gif":
            # Requires pillow
            try:
                writer = PillowWriter(fps=self.FPS)
                anim.save(out_path, writer=writer, dpi=dpi)
                print(f"[SAVED] {out_path}")
            except Exception as e:
                print("[ERROR] GIF save failed. Do you have pillow installed?")
                print("pip install pillow")
                raise e
        else:
            raise ValueError("out_path must end with .mp4 or .gif")

        if show_preview:
            plt.show()
        else:
            plt.close(fig)


if __name__ == "__main__":
    # ============================================================
    # Example: vertical approach only (접근각 90도)
    # V = 0.9 m/s, Vx=Vy=0 => pattern zoom-in only
    # ============================================================
    sim = ApproachPatternSim(Lx=0.25, Ly=0.25, FPS=100)

    # (Optional) world pattern preview
    # sim.preview_world_pattern(Surf_width=8, Surf_height=6, D_cam=1.0)

    # Save the zooming approach video
    sim.save_approach_video(
        D0=1.0,
        Vz=0.9,
        X0=0.0, Vx=0.0,
        Y0=0.0, Vy=0.0,
        D_min=0.08,
        out_path="approach_zoom.mp4",  # or "approach_zoom.gif"
        dpi=160,
        bitrate=6000,
        show_preview=True
    )
