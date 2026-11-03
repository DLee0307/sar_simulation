#!/usr/bin/env python3

import csv
import os
import shutil
import subprocess
import sys
import time

import numpy as np
import rclpy

from sar_msgs.srv import CTRLGetObs


# ============================================================
# PATH SETUP
# ============================================================
BASE_PATH = os.path.expanduser("~/ros2_ws/src/sar_simulation")
PROJECT_PATH = os.path.join(BASE_PATH, "sar_projects", "NewDeepRL")

sys.path.append(BASE_PATH)
sys.path.append(PROJECT_PATH)

from Envs.SAR_Sim_DeepRL_DH import SAR_Sim_DeepRL


# ============================================================
# RESULT FILE
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_CSV_PATH = os.path.join(SCRIPT_DIR, "result.csv")


# ============================================================
# USER-ADJUSTABLE SWEEP SETTINGS
# ============================================================

# 테스트할 접근 속도 목록 [m/s]
SPEED_VALUES = [
#    0.50,	
#    0.75,
#    1.00,
#    1.25,
#    1.50,
#    1.75,
#    2.00,
#    2.25,
#    2.50,
#    2.75,
#    3.00,
#    3.25,
#    3.50,
#    3.75,
#    4.00,
#    4.25,
#    4.50,
#    4.75,
    5.00,



]

# Tau_Cam_GT trigger sweep [s]
TAU_TRIGGER_START = 0.24
TAU_TRIGGER_END = 0.34
TAU_TRIGGER_STEP = 0.01

# 각 속도-Tau 조합을 몇 번 반복할지 설정
REPEATS_PER_CONDITION = 1


# ============================================================
# FIXED TEST SETTINGS
# ============================================================
V_ANGLE_DEG = 90.0             # [deg]
PLANE_ANGLE_DEG = 0.0          # [deg]
A_ROT = -89.9                  # [rad/s^2]

RESET_HEIGHT = 0.4             # [m]

# Policy/sticky pad arm 이후 Gazebo 재생 전 대기시간
START_DELAY_SEC = 1.0          # [s]

# Gazebo 재생 후 착륙 결과 확인까지의 시간
RESULT_CHECK_DELAY_SEC = 1.5   # [s]

# 연속 실험 사이의 대기시간
INTER_TRIAL_DELAY_SEC = 1.0    # [s]


# ============================================================
# COMPLETION / ERROR VOICE ANNOUNCEMENT SETTINGS
# ============================================================
# 정상 완료 또는 중간 오류 발생 시 음성 알림을 반복할 시간
VOICE_ANNOUNCEMENT_DURATION_SEC = 3 * 60   # 3 minutes

# 음성 메시지 반복 간격
VOICE_ANNOUNCEMENT_INTERVAL_SEC = 5.0      # [s]

SUCCESS_VOICE_MESSAGE = "All experiments are finished."
ERROR_VOICE_MESSAGE = (
    "An error has occurred. The experiment was stopped."
)


def find_speech_command() -> str | None:
    """
    Find an available Linux text-to-speech command.

    Search order:
        1. spd-say
        2. espeak-ng
        3. espeak
    """

    for command in ("spd-say", "espeak-ng", "espeak"):
        if shutil.which(command) is not None:
            return command

    return None


def speak_once(
    message: str,
    speech_command: str | None
) -> None:
    """Speak one English status message, or use a terminal fallback."""

    if speech_command is None:
        # Terminal bell fallback when no speech program is installed.
        print(f"\a[VOICE FALLBACK] {message}", flush=True)
        return

    if speech_command == "spd-say":
        command = [
            speech_command,
            "-w",
            "-l",
            "en",
            message,
        ]
    else:
        command = [
            speech_command,
            "-v",
            "en",
            message,
        ]

    try:
        subprocess.run(
            command,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError as exc:
        print(
            f"\a[VOICE ERROR] Could not run "
            f"{speech_command}: {exc}",
            flush=True,
        )


def announce_status_for_three_minutes(
    message: str,
    status_name: str
) -> None:
    """Repeat a status announcement for the configured duration."""

    speech_command = find_speech_command()

    print("\n############################################")
    print(f"{status_name} VOICE ANNOUNCEMENT")
    print("############################################")
    print(f"Message  : {message}")
    print(
        f"Duration : "
        f"{VOICE_ANNOUNCEMENT_DURATION_SEC:.0f} seconds"
    )
    print(
        f"Speech   : "
        f"{speech_command if speech_command else 'terminal fallback'}"
    )
    print("Press Ctrl+C to stop the announcement.")
    print("############################################")

    announcement_start = time.monotonic()

    try:
        while (
            time.monotonic() - announcement_start
            < VOICE_ANNOUNCEMENT_DURATION_SEC
        ):
            speak_once(
                message=message,
                speech_command=speech_command,
            )

            remaining_time = (
                VOICE_ANNOUNCEMENT_DURATION_SEC
                - (time.monotonic() - announcement_start)
            )

            if remaining_time <= 0.0:
                break

            time.sleep(
                min(
                    VOICE_ANNOUNCEMENT_INTERVAL_SEC,
                    remaining_time,
                )
            )

    except KeyboardInterrupt:
        print("\nVoice announcement stopped by user.")


def generate_inclusive_float_range(
    start: float,
    end: float,
    step: float
) -> list[float]:
    """
    Generate floating-point values including the end value.

    Example:
        start=0.30, end=0.50, step=0.02

        [0.30, 0.32, ..., 0.48, 0.50]
    """

    if step <= 0.0:
        raise ValueError(
            "TAU_TRIGGER_STEP must be greater than zero."
        )

    if end < start:
        raise ValueError(
            "TAU_TRIGGER_END must be greater than or equal "
            "to TAU_TRIGGER_START."
        )

    number_of_steps = int(
        round((end - start) / step)
    )

    values = [
        round(start + index * step, 10)
        for index in range(number_of_steps + 1)
    ]

    # Include end when floating-point rounding excluded it.
    if values[-1] < end - 1e-9:
        values.append(round(end, 10))

    return values


def build_trial_plan() -> list[dict]:
    """
    Build every speed, Tau-trigger and repeat combination.

    Order:
        speed
          -> Tau trigger
             -> repeat number
    """

    tau_trigger_values = generate_inclusive_float_range(
        start=TAU_TRIGGER_START,
        end=TAU_TRIGGER_END,
        step=TAU_TRIGGER_STEP
    )

    trial_plan = []

    for v_mag in SPEED_VALUES:
        for tau_trigger in tau_trigger_values:
            for repeat_number in range(
                1,
                REPEATS_PER_CONDITION + 1
            ):
                trial_plan.append({
                    "v_mag": float(v_mag),
                    "tau_cam_gt_trigger": float(tau_trigger),
                    "repeat_number": int(repeat_number),
                })

    return trial_plan


def initialize_result_csv() -> None:
    """
    Create result.csv with its header if necessary.

    Existing results are preserved. New results are appended.
    """

    file_needs_header = (
        not os.path.exists(RESULT_CSV_PATH)
        or os.path.getsize(RESULT_CSV_PATH) == 0
    )

    with open(
        RESULT_CSV_PATH,
        mode="a",
        newline="",
        encoding="utf-8"
    ) as csv_file:
        writer = csv.writer(csv_file)

        if file_needs_header:
            writer.writerow([
                "v_mag",
                "tau_cam_gt",
                "Leg",
            ])

    print(f"[CSV] Result file ready: {RESULT_CSV_PATH}")


def calculate_leg_score(
    body_contact: bool,
    attached_leg_count: int
) -> float:
    """
    Landing score:

        3-4 legs, no body contact -> 1.00
        3-4 legs, body contact    -> 0.75
        1-2 legs, no body contact -> 0.50
        1-2 legs, body contact    -> 0.25
        0 legs                    -> 0.00
    """

    if attached_leg_count <= 0:
        return 0.0

    if attached_leg_count <= 2:
        return 0.25 if body_contact else 0.5

    return 0.75 if body_contact else 1.0


def append_result_to_csv(
    v_mag: float,
    tau_cam_gt_trigger: float,
    leg_score: float
) -> None:
    """
    Append one trial result to result.csv.
    """

    with open(
        RESULT_CSV_PATH,
        mode="a",
        newline="",
        encoding="utf-8"
    ) as csv_file:
        writer = csv.writer(csv_file)

        writer.writerow([
            round(float(v_mag), 4),
            round(float(tau_cam_gt_trigger), 4),
            round(float(leg_score), 2),
        ])

    print(
        f"[CSV] Saved: "
        f"v_mag={v_mag:.3f}, "
        f"tau_cam_gt={tau_cam_gt_trigger:.3f}, "
        f"Leg={leg_score:.2f}"
    )


def get_tau_cam_gt(env) -> float:
    """
    Read the current unscaled Tau_Cam_GT.
    """

    response = env.callService(
        "/CTRL/Get_Obs",
        CTRLGetObs.Request(),
        CTRLGetObs
    )

    if response is None:
        return np.nan

    return float(response.tau_cam_gt)


def prepare_fixed_flight(
    env,
    v_mag: float,
    tau_cam_gt_trigger: float,
    trial_number: int,
    total_trials: int,
    repeat_number: int
) -> None:
    """
    Reset and prepare one fixed flight condition.
    """

    print("\n\n############################################")
    print(
        f"Starting trial {trial_number} / {total_trials}"
    )
    print("############################################")
    print(f"V_mag       : {v_mag:.3f} m/s")
    print(f"Tau trigger : {tau_cam_gt_trigger:.3f} s")
    print(
        f"Repeat      : "
        f"{repeat_number} / {REPEATS_PER_CONDITION}"
    )
    print("############################################")

    # --------------------------------------------------------
    # 1. PAUSE SIMULATION
    # --------------------------------------------------------
    if not env.pausePhysics(True):
        raise RuntimeError(
            "Failed to pause Gazebo physics."
        )

    env.start_time_real = time.time()

    # --------------------------------------------------------
    # 2. RESET DRONE AND CONTROLLER
    # --------------------------------------------------------
    reset_ok = env.resetPose(
        z_0=RESET_HEIGHT
    )

    if not reset_ok:
        raise RuntimeError(
            "resetPose() failed."
        )

    # Reset Python-side trigger, impact and episode values.
    env._resetParams()

    # --------------------------------------------------------
    # 3. SET FIXED FLIGHT CONDITION
    # --------------------------------------------------------
    env.V_mag_range = [
        v_mag,
        v_mag,
    ]

    env.V_angle_range = [
        V_ANGLE_DEG,
        V_ANGLE_DEG,
    ]

    env.Plane_Angle_range = [
        PLANE_ANGLE_DEG,
        PLANE_ANGLE_DEG,
    ]

    env.Fine_Tuning_Flag = False

    env.V_mag = v_mag
    env.V_angle = V_ANGLE_DEG

    env.Plane_Angle_deg = PLANE_ANGLE_DEG
    env.Plane_Angle_rad = np.deg2rad(
        PLANE_ANGLE_DEG
    )

    plane_position = np.asarray(
        getattr(
            env,
            "r_P_O",
            [np.nan, np.nan, np.nan]
        ),
        dtype=float
    )

    if (
        plane_position.shape != (3,)
        or not np.all(np.isfinite(plane_position))
    ):
        env.r_P_O = [
            3.0,
            0.0,
            2.5,
        ]

    # --------------------------------------------------------
    # 4. CREATE INITIAL VELOCITY STATE
    # --------------------------------------------------------
    initial_ok = env._initialStep()

    if not initial_ok:
        raise RuntimeError(
            "_initialStep() failed."
        )

    # Keep Gazebo paused while sticky pads and policy are armed.
    if not env.pausePhysics(True):
        raise RuntimeError(
            "Failed to pause Gazebo after _initialStep()."
        )

    for _ in range(5):
        rclpy.spin_once(
            env,
            timeout_sec=0.02
        )

    tau_initial = get_tau_cam_gt(env)

    print("\n============================================")
    print("Initial flight condition prepared")
    print("============================================")
    print(f"Trial        : {trial_number}/{total_trials}")
    print(f"V_mag        : {env.V_mag:.3f} m/s")
    print(f"V_angle      : {env.V_angle:.3f} deg")
    print(f"Plane angle  : {env.Plane_Angle_deg:.3f} deg")
    print(f"Tau_Cam_GT   : {tau_initial:.4f} s")
    print(f"Tau threshold: {tau_cam_gt_trigger:.4f} s")
    print("Gazebo       : PAUSED")
    print("============================================")

    if not np.isfinite(tau_initial):
        raise RuntimeError(
            "Initial Tau_Cam_GT is NaN or infinite."
        )

    if tau_initial <= tau_cam_gt_trigger:
        raise RuntimeError(
            f"Initial Tau_Cam_GT={tau_initial:.4f} s is already "
            f"below trigger threshold "
            f"{tau_cam_gt_trigger:.4f} s."
        )


def arm_landing_system(
    env,
    tau_cam_gt_trigger: float
) -> None:
    """
    Enable sticky pads and arm the MANUAL_DH controller.
    """

    print("\n============================================")
    print("Arming landing system")
    print("============================================")

    # --------------------------------------------------------
    # 1. ENABLE STICKY PADS
    # --------------------------------------------------------
    sticky_ok = env.sendCmd(
        "GZ_StickyPads",
        cmd_vals=[
            1.0,
            1.0,
            1.0,
        ],
        cmd_flag=1.0
    )

    if not sticky_ok:
        raise RuntimeError(
            "Failed to enable Gazebo sticky pads."
        )

    time.sleep(0.5)

    print("[ARM] Sticky pads enabled")

    # --------------------------------------------------------
    # 2. ARM MANUAL_DH POLICY
    # --------------------------------------------------------
    policy_ok = env.sendCmd(
        "Policy",
        cmd_vals=[
            tau_cam_gt_trigger,
            A_ROT,
            A_ROT,
        ],
        cmd_flag=A_ROT
    )

    if not policy_ok:
        raise RuntimeError(
            "Failed to arm the MANUAL_DH policy."
        )

    time.sleep(0.5)

    print(
        f"[ARM] MANUAL_DH policy armed\n"
        f"      Tau_Cam_GT trigger : "
        f"{tau_cam_gt_trigger:.3f} s\n"
        f"      Flip acceleration  : "
        f"{A_ROT:.1f} rad/s^2"
    )

    print("============================================")


def disable_sticky_pads(env) -> None:
    """
    Disable sticky pads before resetting for the next trial.
    """

    try:
        sticky_ok = env.sendCmd(
            "GZ_StickyPads",
            cmd_vals=[
                1.0,
                1.0,
                1.0,
            ],
            cmd_flag=0.0
        )

        if sticky_ok:
            print("[RESET] Sticky pads disabled")
        else:
            print(
                "[WARN] Sticky-pad disable command failed"
            )

    except Exception as exc:
        print(
            f"[WARN] Sticky-pad disable error: {exc}"
        )

    time.sleep(0.5)


def run_and_evaluate_trial(
    env,
    v_mag: float,
    tau_cam_gt_trigger: float,
    trial_number: int,
    total_trials: int
) -> float:
    """
    Run one landing trial, evaluate it and save the result.
    """

    print(
        f"\nWaiting {START_DELAY_SEC:.1f} second before "
        "resuming the simulation..."
    )

    time.sleep(START_DELAY_SEC)

    # --------------------------------------------------------
    # RESUME SIMULATION
    # --------------------------------------------------------
    if not env.pausePhysics(False):
        raise RuntimeError(
            "Failed to resume Gazebo physics."
        )

    print("\n============================================")
    print(
        f"Trial {trial_number}/{total_trials} resumed"
    )
    print("============================================")
    print(f"Speed       : {v_mag:.2f} m/s")
    print(f"Approach    : {V_ANGLE_DEG:.1f} deg")
    print(f"Tau trigger : {tau_cam_gt_trigger:.3f} s")
    print(f"a_Rot       : {A_ROT:.1f} rad/s^2")
    print("Sticky pads : ON")
    print("============================================")

    print(
        f"\nRunning trial for "
        f"{RESULT_CHECK_DELAY_SEC:.1f} seconds..."
    )

    # --------------------------------------------------------
    # RUN AND RECEIVE CONTACT MESSAGES
    # --------------------------------------------------------
    trial_start_time = time.monotonic()

    while (
        rclpy.ok()
        and time.monotonic() - trial_start_time
        < RESULT_CHECK_DELAY_SEC
    ):
        rclpy.spin_once(
            env,
            timeout_sec=0.02
        )

        time.sleep(0.005)

    # --------------------------------------------------------
    # FREEZE RESULT
    # --------------------------------------------------------
    if not env.pausePhysics(True):
        raise RuntimeError(
            "Failed to pause Gazebo before checking result."
        )

    for _ in range(10):
        rclpy.spin_once(
            env,
            timeout_sec=0.02
        )

    # --------------------------------------------------------
    # READ LANDING RESULT
    # --------------------------------------------------------
    body_contact = bool(
        getattr(
            env,
            "BodyContact_Flag",
            False
        )
    )

    pad_contacts = [
        bool(getattr(env, "Pad1_Contact", False)),
        bool(getattr(env, "Pad2_Contact", False)),
        bool(getattr(env, "Pad3_Contact", False)),
        bool(getattr(env, "Pad4_Contact", False)),
    ]

    attached_leg_count = int(
        sum(pad_contacts)
    )

    reported_pad_connections = getattr(
        env,
        "Pad_Connections",
        np.nan
    )

    leg_score = calculate_leg_score(
        body_contact=body_contact,
        attached_leg_count=attached_leg_count
    )

    # --------------------------------------------------------
    # PRINT RESULT
    # --------------------------------------------------------
    print("\n============================================")
    print(
        f"Trial {trial_number}/{total_trials} result"
    )
    print("============================================")

    print(
        f"Body contact : "
        f"{'YES' if body_contact else 'NO'}"
    )

    for leg_index, is_attached in enumerate(
        pad_contacts,
        start=1
    ):
        print(
            f"Leg {leg_index} attached : "
            f"{'YES' if is_attached else 'NO'}"
        )

    print("--------------------------------------------")

    print(
        f"Total attached legs : "
        f"{attached_leg_count} / 4"
    )

    if np.isfinite(reported_pad_connections):
        print(
            f"Pad_Connections value : "
            f"{int(reported_pad_connections)}"
        )
    else:
        print(
            "Pad_Connections value : unavailable"
        )

    print(f"Leg score : {leg_score:.2f}")
    print("============================================")

    # --------------------------------------------------------
    # SAVE RESULT
    # --------------------------------------------------------
    append_result_to_csv(
        v_mag=v_mag,
        tau_cam_gt_trigger=tau_cam_gt_trigger,
        leg_score=leg_score
    )

    return leg_score


def main(args=None):
    env = None
    rclpy_initialized = False
    all_trials_completed = False
    pending_exception = None

    try:
        initialize_result_csv()

        trial_plan = build_trial_plan()
        total_trials = len(trial_plan)

        if total_trials <= 0:
            raise RuntimeError(
                "The trial plan is empty. Check SPEED_VALUES, "
                "Tau settings and REPEATS_PER_CONDITION."
            )

        tau_values = generate_inclusive_float_range(
            start=TAU_TRIGGER_START,
            end=TAU_TRIGGER_END,
            step=TAU_TRIGGER_STEP
        )

        print("\n############################################")
        print("Sweep configuration")
        print("############################################")
        print(f"Speeds      : {SPEED_VALUES}")
        print(f"Tau values  : {tau_values}")
        print(
            f"Repeats     : {REPEATS_PER_CONDITION} "
            "per condition"
        )
        print(f"Total trials: {total_trials}")
        print("############################################")

        rclpy.init(args=args)
        rclpy_initialized = True

        env = SAR_Sim_DeepRL(
            Ang_Acc_range=[
                A_ROT,
                A_ROT,
            ],
            V_mag_range=[
                min(SPEED_VALUES),
                max(SPEED_VALUES),
            ],
            V_angle_range=[
                V_ANGLE_DEG,
                V_ANGLE_DEG,
            ],
            Plane_Angle_range=[
                PLANE_ANGLE_DEG,
                PLANE_ANGLE_DEG,
            ],
            Render=True,
            Fine_Tune=False,
            GZ_Timeout=False,
            Obs_Mode="TauTheta"
        )

        if env.Policy_Type != "MANUAL_DH":
            raise RuntimeError(
                f"Policy_Type is '{env.Policy_Type}', but this test "
                "requires Policy_Type='MANUAL_DH'."
            )

        for trial_index, trial in enumerate(
            trial_plan,
            start=1
        ):
            v_mag = trial["v_mag"]
            tau_cam_gt_trigger = trial[
                "tau_cam_gt_trigger"
            ]
            repeat_number = trial["repeat_number"]

            # 1. Reset and prepare this trial.
            prepare_fixed_flight(
                env=env,
                v_mag=v_mag,
                tau_cam_gt_trigger=tau_cam_gt_trigger,
                trial_number=trial_index,
                total_trials=total_trials,
                repeat_number=repeat_number
            )

            # 2. Enable sticky pads and arm trigger.
            arm_landing_system(
                env=env,
                tau_cam_gt_trigger=tau_cam_gt_trigger
            )

            # 3. Run, evaluate and save result.
            run_and_evaluate_trial(
                env=env,
                v_mag=v_mag,
                tau_cam_gt_trigger=tau_cam_gt_trigger,
                trial_number=trial_index,
                total_trials=total_trials
            )

            # ------------------------------------------------
            # PREPARE FOR NEXT TRIAL
            # ------------------------------------------------
            if trial_index < total_trials:
                disable_sticky_pads(env)

                print(
                    f"\nWaiting {INTER_TRIAL_DELAY_SEC:.1f} second "
                    "before the next trial..."
                )

                time.sleep(
                    INTER_TRIAL_DELAY_SEC
                )

        # This flag is set only after every trial finishes normally.
        all_trials_completed = True

        print("\n############################################")
        print("All sweep trials completed")
        print("############################################")
        print(f"Total trials : {total_trials}")
        print(f"CSV file     : {RESULT_CSV_PATH}")
        print("############################################")

    except KeyboardInterrupt:
        print(
            "\n\n[ERROR] KeyboardInterrupt received. "
            "The experiment stopped before completion."
        )

    except Exception as exc:
        pending_exception = exc
        print(f"\n[ERROR] {type(exc).__name__}: {exc}")

    finally:
        # ----------------------------------------------------
        # SAFELY STOP GAZEBO / ROS BEFORE VOICE ANNOUNCEMENT
        # ----------------------------------------------------
        if env is not None:
            try:
                env.pausePhysics(True)
            except Exception as exc:
                print(f"Pause error: {exc}")

            try:
                env.destroy()
            except Exception as exc:
                print(
                    f"Environment destroy error: {exc}"
                )

        if rclpy_initialized and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as exc:
                print(f"ROS shutdown error: {exc}")

        # ----------------------------------------------------
        # ENGLISH STATUS ANNOUNCEMENT
        # ----------------------------------------------------
        if all_trials_completed:
            announce_status_for_three_minutes(
                message=SUCCESS_VOICE_MESSAGE,
                status_name="SUCCESS",
            )
        else:
            announce_status_for_three_minutes(
                message=ERROR_VOICE_MESSAGE,
                status_name="ERROR",
            )

    # Preserve the original nonzero/error behavior after announcing.
    if pending_exception is not None:
        raise pending_exception


if __name__ == "__main__":
    main()
