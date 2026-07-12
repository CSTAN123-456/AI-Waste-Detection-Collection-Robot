from pathlib import Path
import os
import time

import cv2
import serial
from ultralytics import YOLO


# ============================================================
# MODEL SETTINGS
# ============================================================

BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "models" / "best.pt"


# ============================================================
# CAMERA SETTINGS
# ============================================================

CAMERA_INDEX = 1

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

WINDOW_NAME = "FYP Waste Detection Robot"


# ============================================================
# ESP32 SERIAL SETTINGS
# ============================================================

ESP32_PORT = "COM4"
ESP32_BAUD_RATE = 115200

SERIAL_RESPONSE_TIMEOUT_SECONDS = 2.0
PICKUP_TIMEOUT_SECONDS = 60.0


# ============================================================
# AI DETECTION SETTINGS
# ============================================================

CONFIDENCE_THRESHOLD = 0.60

ALLOWED_CLASSES = {
    "CAN",
    "BOTTLE",
}

CLASS_NAME_ALIASES = {
    "PLASTIC-BOTTLE": "BOTTLE",
    "PLASTIC_BOTTLE": "BOTTLE",
    "PLASTIC BOTTLE": "BOTTLE",
    "BOTTLE": "BOTTLE",
    "CAN": "CAN",
}

CENTER_LEFT_RATIO = 0.48
CENTER_RIGHT_RATIO = 0.52

# How often Python sends a command to the ESP32.
COMMAND_INTERVAL_SECONDS = 0.20


# ============================================================
# TARGET CONFIRMATION SETTINGS
# ============================================================

TARGET_CONFIRM_FRAMES = 3

TARGET_MATCH_DISTANCE_RATIO = 0.20

# Robot stops when a locked target disappears temporarily.
TARGET_LOST_HOLD_SECONDS = 0.75


# ============================================================
# POST-PICKUP STOP
# ============================================================

POST_PICKUP_STOP_SECONDS = 2.0


# ============================================================
# GET YOLO CLASS NAME
# ============================================================

def get_class_name(model, class_id):
    names = model.names

    if isinstance(names, dict):
        raw_name = str(names[class_id])
    else:
        raw_name = str(names[class_id])

    normalized_name = raw_name.strip().upper()

    return CLASS_NAME_ALIASES.get(
        normalized_name,
        normalized_name
    )


# ============================================================
# DETERMINE TARGET DIRECTION
# ============================================================

def get_direction(center_x, frame_width):
    horizontal_ratio = center_x / frame_width

    if horizontal_ratio < CENTER_LEFT_RATIO:
        return "LEFT"

    if horizontal_ratio > CENTER_RIGHT_RATIO:
        return "RIGHT"

    return "CENTER"


# ============================================================
# READ ONE SERIAL RESPONSE
# ============================================================

def read_serial_response(
    esp32,
    timeout_seconds=SERIAL_RESPONSE_TIMEOUT_SECONDS
):
    end_time = time.time() + timeout_seconds

    while time.time() < end_time:
        if esp32.in_waiting > 0:
            raw_line = esp32.readline()

            line = raw_line.decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if line:
                return line

        time.sleep(0.01)

    return ""


# ============================================================
# SEND COMMAND TO ESP32
# ============================================================

def send_command(esp32, command):
    message = command.strip() + "\n"

    esp32.write(
        message.encode("utf-8")
    )

    esp32.flush()

    print(f"PYTHON -> ESP32: {command}")

    response = read_serial_response(esp32)

    if response:
        print(f"ESP32 -> PYTHON: {response}")
    else:
        print("ESP32 -> PYTHON: NO RESPONSE")

    return response


# ============================================================
# DRAW CAMERA STATUS PANEL
# ============================================================

def draw_status_panel(
    frame,
    mode_text,
    detail_text,
    serial_text=""
):
    cv2.rectangle(
        frame,
        (10, 10),
        (830, 125),
        (0, 0, 0),
        -1
    )

    cv2.putText(
        frame,
        f"MODE: {mode_text}",
        (25, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        detail_text,
        (25, 77),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2
    )

    if serial_text:
        cv2.putText(
            frame,
            serial_text[:105],
            (25, 108),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (200, 200, 200),
            1
        )


# ============================================================
# WAIT FOR PICKUP TO FINISH
# ============================================================

def wait_for_pickup_done(
    esp32,
    frozen_frame
):
    print("\nPickup sequence started.")
    print("Waiting for PICKUP_DONE...\n")

    start_time = time.time()
    quit_requested = False

    while (
        time.time() - start_time <
        PICKUP_TIMEOUT_SECONDS
    ):
        display_frame = frozen_frame.copy()

        cv2.rectangle(
            display_frame,
            (10, 10),
            (830, 100),
            (0, 0, 0),
            -1
        )

        cv2.putText(
            display_frame,
            "MODE: PICKUP IN PROGRESS",
            (25, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (0, 255, 255),
            2
        )

        cv2.putText(
            display_frame,
            "Drive stopped; outer sensors ignored",
            (25, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (255, 255, 255),
            2
        )

        cv2.imshow(
            WINDOW_NAME,
            display_frame
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            quit_requested = True

        if esp32.in_waiting > 0:
            raw_line = esp32.readline()

            line = raw_line.decode(
                "utf-8",
                errors="ignore"
            ).strip()

            if not line:
                continue

            print(f"ESP32: {line}")

            if line == "PICKUP_DONE":
                print("\nPickup sequence completed.\n")

                return True, quit_requested

        time.sleep(0.01)

    print("\nERROR: Pickup completion timed out.\n")

    return False, quit_requested


# ============================================================
# CLEAR OLD CAMERA FRAMES
# ============================================================

def clear_camera_buffer(
    camera,
    frame_count=10
):
    for _ in range(frame_count):
        camera.grab()

        time.sleep(0.01)


# ============================================================
# EXTRACT VALID YOLO DETECTIONS
# ============================================================

def extract_detections(
    result,
    model,
    frame_width
):
    detections = []

    if result.boxes is None:
        return detections

    for box in result.boxes:
        confidence = float(
            box.conf[0].item()
        )

        if confidence < CONFIDENCE_THRESHOLD:
            continue

        class_id = int(
            box.cls[0].item()
        )

        object_name = get_class_name(
            model,
            class_id
        )

        if object_name not in ALLOWED_CLASSES:
            continue

        x1, y1, x2, y2 = (
            box.xyxy[0]
            .cpu()
            .tolist()
        )

        center_x = int(
            (x1 + x2) / 2
        )

        center_y = int(
            (y1 + y2) / 2
        )

        direction = get_direction(
            center_x,
            frame_width
        )

        detections.append(
            {
                "name": object_name,
                "confidence": confidence,
                "center_x": center_x,
                "center_y": center_y,
                "direction": direction,
                "x1": int(x1),
                "y1": int(y1),
                "x2": int(x2),
                "y2": int(y2),
            }
        )

    return detections


# ============================================================
# CHOOSE A NEW TARGET
# ============================================================

def choose_initial_target(detections):
    if not detections:
        return None

    return max(
        detections,
        key=lambda item: item["confidence"]
    )


# ============================================================
# FIND THE CURRENT LOCKED TARGET
# ============================================================

def find_locked_target(
    detections,
    locked_name,
    previous_center_x
):
    matching_detections = [
        item
        for item in detections
        if item["name"] == locked_name
    ]

    if not matching_detections:
        return None

    return min(
        matching_detections,
        key=lambda item: abs(
            item["center_x"] -
            previous_center_x
        )
    )


# ============================================================
# CREATE A CLEAN TARGET STATE
# ============================================================

def reset_target_state():
    return {
        "candidate_name": None,
        "candidate_center_x": None,
        "candidate_frame_count": 0,
        "target_locked": False,
        "locked_name": None,
        "locked_center_x": None,
        "last_locked_target_seen_time": 0.0,
    }


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"YOLO model was not found:\n{MODEL_PATH}"
        )

    print("Loading YOLO model...")

    model = YOLO(
        str(MODEL_PATH)
    )

    print("YOLO model loaded.")
    print("MODEL CLASS NAMES", model.names)

    # --------------------------------------------------------
    # OPEN ESP32 SERIAL CONNECTION
    # --------------------------------------------------------

    try:
        esp32 = serial.Serial(
            port=ESP32_PORT,
            baudrate=ESP32_BAUD_RATE,
            timeout=0.1,
            write_timeout=2
        )

    except serial.SerialException as error:
        print("\nCould not open the ESP32 serial port.")
        print(f"Port: {ESP32_PORT}")
        print(f"Error: {error}")

        print(
            "\nClose Arduino Serial Monitor and verify "
            "the ESP32 COM port."
        )

        return

    print(
        f"Connected to ESP32 on "
        f"{ESP32_PORT} at "
        f"{ESP32_BAUD_RATE} baud."
    )

    # Allow the ESP32 and servos to initialise.
    time.sleep(3.5)

    # Remove old ESP32 startup messages.
    while esp32.in_waiting > 0:
        startup_line = (
            esp32.readline()
            .decode(
                "utf-8",
                errors="ignore"
            )
            .strip()
        )

        if startup_line:
            print(
                f"ESP32 STARTUP: {startup_line}"
            )

    # --------------------------------------------------------
    # OPEN CAMERA
    # --------------------------------------------------------

    if os.name == "nt":
        camera = cv2.VideoCapture(
            CAMERA_INDEX,
            cv2.CAP_DSHOW
        )
    else:
        camera = cv2.VideoCapture(
            CAMERA_INDEX
        )

    camera.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        CAMERA_WIDTH
    )

    camera.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        CAMERA_HEIGHT
    )

    if not camera.isOpened():
        print(
            f"Could not open camera index "
            f"{CAMERA_INDEX}."
        )

        esp32.close()

        return

    print("Camera opened successfully.")
    print("Press Q in the camera window to stop.")

    state = reset_target_state()

    post_pickup_stop_until = 0.0

    last_command_time = 0.0
    last_serial_response = ""

    quit_requested = False

    try:
        while not quit_requested:
            success, frame = camera.read()

            if not success:
                print(
                    "Camera frame could not be read."
                )

                break

            _, frame_width = frame.shape[:2]

            # ------------------------------------------------
            # RUN YOLO
            # ------------------------------------------------

            result = model.predict(
                source=frame,
                conf=CONFIDENCE_THRESHOLD,
                verbose=False
            )[0]

            # Keep bounding boxes, labels and confidence visible.
            annotated_frame = result.plot()

            detections = extract_detections(
                result,
                model,
                frame_width
            )

            current_time = time.time()

            post_pickup_stop_active = (
                current_time <
                post_pickup_stop_until
            )

            visible_locked_target = None

            mode_text = "ROAMING"

            detail_text = (
                "Continuous forward movement; "
                "outer sensors active"
            )

            # ------------------------------------------------
            # POST-PICKUP TWO-SECOND STOP
            # ------------------------------------------------

            if post_pickup_stop_active:
                remaining_time = max(
                    0.0,
                    post_pickup_stop_until -
                    current_time
                )

                state = reset_target_state()

                mode_text = "POST-PICKUP STOP"

                detail_text = (
                    f"Robot stopped for "
                    f"{remaining_time:.1f} seconds"
                )

            # ------------------------------------------------
            # CURRENT TARGET IS LOCKED
            # ------------------------------------------------

            elif state["target_locked"]:
                visible_locked_target = (
                    find_locked_target(
                        detections,
                        state["locked_name"],
                        state["locked_center_x"]
                    )
                )

                if visible_locked_target is not None:
                    state["locked_center_x"] = (
                        visible_locked_target[
                            "center_x"
                        ]
                    )

                    state[
                        "last_locked_target_seen_time"
                    ] = current_time

                    mode_text = "TARGET TRACKING"

                    detail_text = (
                        f'{state["locked_name"]} | '
                        f'{visible_locked_target["direction"]} | '
                        f'{visible_locked_target["confidence"]:.2f}'
                    )

                else:
                    time_missing = (
                        current_time -
                        state[
                            "last_locked_target_seen_time"
                        ]
                    )

                    if (
                        time_missing <=
                        TARGET_LOST_HOLD_SECONDS
                    ):
                        mode_text = (
                            "TARGET TEMPORARILY LOST"
                        )

                        detail_text = (
                            f"Robot stopped | "
                            f"{time_missing:.2f} / "
                            f"{TARGET_LOST_HOLD_SECONDS:.2f} s"
                        )

                    else:
                        print(
                            "Target lost. "
                            "Returning to roaming."
                        )

                        state = reset_target_state()

                        mode_text = "ROAMING"

                        detail_text = (
                            "Target lost; "
                            "outer sensors active"
                        )

            # ------------------------------------------------
            # SEARCH FOR A NEW TARGET
            # ------------------------------------------------

            if (
                not post_pickup_stop_active
                and
                not state["target_locked"]
            ):
                selected_detection = (
                    choose_initial_target(
                        detections
                    )
                )

                if selected_detection is None:
                    state["candidate_name"] = None
                    state["candidate_center_x"] = None
                    state["candidate_frame_count"] = 0

                else:
                    selected_name = (
                        selected_detection["name"]
                    )

                    selected_center_x = (
                        selected_detection[
                            "center_x"
                        ]
                    )

                    maximum_match_distance = (
                        frame_width *
                        TARGET_MATCH_DISTANCE_RATIO
                    )

                    same_candidate = (
                        state["candidate_name"] ==
                        selected_name
                        and
                        state["candidate_center_x"]
                        is not None
                        and
                        abs(
                            selected_center_x -
                            state["candidate_center_x"]
                        ) <=
                        maximum_match_distance
                    )

                    if same_candidate:
                        state[
                            "candidate_frame_count"
                        ] += 1
                    else:
                        state["candidate_name"] = (
                            selected_name
                        )

                        state[
                            "candidate_frame_count"
                        ] = 1

                    state["candidate_center_x"] = (
                        selected_center_x
                    )

                    mode_text = "VERIFYING TARGET"

                    detail_text = (
                        f'{state["candidate_name"]} | '
                        f'{state["candidate_frame_count"]}/'
                        f'{TARGET_CONFIRM_FRAMES}'
                    )

                    if (
                        state["candidate_frame_count"] >=
                        TARGET_CONFIRM_FRAMES
                    ):
                        state["target_locked"] = True

                        state["locked_name"] = (
                            selected_name
                        )

                        state["locked_center_x"] = (
                            selected_center_x
                        )

                        state[
                            "last_locked_target_seen_time"
                        ] = current_time

                        visible_locked_target = (
                            selected_detection
                        )

                        print(
                            f"Target locked: "
                            f"{state['locked_name']}"
                        )

                        mode_text = "TARGET TRACKING"

                        detail_text = (
                            f'{state["locked_name"]} | '
                            f'{selected_detection["direction"]} | '
                            f'{selected_detection["confidence"]:.2f}'
                        )

            # ------------------------------------------------
            # COMMAND TIMING
            # ------------------------------------------------

            command_due = (
                current_time -
                last_command_time >=
                COMMAND_INTERVAL_SECONDS
            )

            # ------------------------------------------------
            # POST-PICKUP STOP
            # ------------------------------------------------

            if (
                post_pickup_stop_active
                and
                command_due
            ):
                response = send_command(
                    esp32,
                    "STOP"
                )

                last_command_time = time.time()
                last_serial_response = response

            # ------------------------------------------------
            # TEMPORARY TARGET LOSS
            # ------------------------------------------------

            elif (
                state["target_locked"]
                and
                visible_locked_target is None
                and
                command_due
            ):
                response = send_command(
                    esp32,
                    "STOP"
                )

                last_command_time = time.time()
                last_serial_response = response

            # ------------------------------------------------
            # SEND TARGET COMMAND
            # ------------------------------------------------

            elif (
                state["target_locked"]
                and
                visible_locked_target is not None
                and
                command_due
            ):
                target = visible_locked_target

                command = (
                    f'TARGET,'
                    f'{target["name"]},'
                    f'{target["direction"]},'
                    f'{target["center_x"]},'
                    f'{frame_width},'
                    f'{target["confidence"]:.3f}'
                )

                response = send_command(
                    esp32,
                    command
                )

                last_command_time = time.time()
                last_serial_response = response

                # ESP32 begins the complete arm sequence.
                if "PICKUP_START" in response:
                    pickup_frame = (
                        annotated_frame.copy()
                    )

                    (
                        pickup_complete,
                        pickup_quit
                    ) = wait_for_pickup_done(
                        esp32,
                        pickup_frame
                    )

                    state = reset_target_state()

                    visible_locked_target = None

                    last_command_time = 0.0

                    clear_camera_buffer(
                        camera,
                        frame_count=10
                    )

                    if pickup_complete:
                        last_serial_response = (
                            "PICKUP_DONE"
                        )

                        post_pickup_stop_until = (
                            time.time() +
                            POST_PICKUP_STOP_SECONDS
                        )

                        print(
                            "Post-pickup stop started: "
                            f"{POST_PICKUP_STOP_SECONDS:.1f} seconds."
                        )

                        mode_text = "POST-PICKUP STOP"

                        detail_text = (
                            "Drive stopped; "
                            "detections temporarily ignored"
                        )

                    else:
                        last_serial_response = (
                            "PICKUP TIMEOUT"
                        )

                    if pickup_quit:
                        quit_requested = True

            # ------------------------------------------------
            # ROAMING COMMAND
            #
            # The ESP32 handles continuous movement and outer
            # ultrasonic scanning after receiving NONE.
            # ------------------------------------------------

            elif (
                not state["target_locked"]
                and
                command_due
            ):
                response = send_command(
                    esp32,
                    "NONE"
                )

                last_command_time = time.time()
                last_serial_response = response

            # ------------------------------------------------
            # DISPLAY CAMERA WINDOW
            # ------------------------------------------------

            draw_status_panel(
                annotated_frame,
                mode_text,
                detail_text,
                last_serial_response
            )

            cv2.imshow(
                WINDOW_NAME,
                annotated_frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                quit_requested = True

    except KeyboardInterrupt:
        print(
            "\nProgram stopped with Ctrl+C."
        )

    finally:
        print(
            "Stopping robot and closing connections..."
        )

        try:
            if esp32.is_open:
                send_command(
                    esp32,
                    "STOP"
                )

        except Exception:
            pass

        camera.release()

        cv2.destroyAllWindows()

        if esp32.is_open:
            esp32.close()

        print("Program closed safely.")


# ============================================================
# PROGRAM ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()