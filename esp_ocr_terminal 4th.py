"""
ESP32-S3 Camera OCR - LIVE VIDEO + TERMINAL + ESP32 TFT SCREEN
--------------------------------------------------------------
* A live video window shows what the ESP32 camera sees (the same picture is
  also drawn on the ESP32's own TFT screen).
* Click the video window, then press SPACE (or click the video) to take a photo.
* The photo is read with EasyOCR. The terminal then asks if you want to edit
  the text and whether to search it on Google.
* The captured photo and the text that was read are also shown on the TFT.

Install:
    pip install pyserial easyocr opencv-python numpy

Run:
    python esp_ocr_terminal.py --port COM6

Keys in the video window:
    SPACE / ENTER / mouse click = capture and read text
    Q or ESC                    = quit

IMPORTANT: close the Arduino Serial Monitor first, or the port will be busy.
"""

import argparse
import os
import platform
import shlex
import shutil
import struct
import subprocess
import sys
import threading
import time
import warnings
import webbrowser
from datetime import datetime
from urllib.parse import quote_plus

warnings.filterwarnings("ignore")

LANGUAGES = ["en"]          # e.g. ["en", "hi"]  (the TFT can only show plain English letters)
USE_GPU = False
SAVE_DIR = "captures"       # photos are also saved on the laptop here
MAX_QUERY_LEN = 300
MAX_IMAGE_BYTES = 5_000_000
MAX_TFT_CHARS = 400         # longest text sent to the TFT screen
PREVIEW_SCALE = 2           # the live preview is 320x240; show it 2x bigger on the laptop
WINDOW = "ESP32 Live Camera"

cv2 = None   # loaded inside run()
np = None


def load_cv():
    global cv2, np
    import cv2 as _cv2
    import numpy as _np
    cv2, np = _cv2, _np


# ------------------- open in a dedicated terminal window -------------------
def relaunch_in_terminal():
    """Open this script in its own terminal window, then exit this process."""
    if "--in-terminal" in sys.argv:
        return
    script = os.path.abspath(__file__)
    args = [sys.executable, script, "--in-terminal"] + sys.argv[1:]
    system = platform.system()
    try:
        if system == "Windows":
            subprocess.Popen(args, creationflags=subprocess.CREATE_NEW_CONSOLE)
        elif system == "Darwin":
            cmd = " ".join(shlex.quote(a) for a in args)
            cmd = cmd.replace("\\", "\\\\").replace('"', '\\"')
            subprocess.Popen(
                ["osascript", "-e", f'tell application "Terminal" to do script "{cmd}"']
            )
        else:
            launched = False
            for term in (
                ["gnome-terminal", "--"],
                ["x-terminal-emulator", "-e"],
                ["konsole", "-e"],
                ["xterm", "-e"],
            ):
                if shutil.which(term[0]):
                    subprocess.Popen(term + args)
                    launched = True
                    break
            if not launched:
                return
        sys.exit(0)
    except OSError:
        return


# ------------------------------ serial helpers ------------------------------
def find_port():
    import serial.tools.list_ports

    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid == 0x303A:  # Espressif native USB
            return p.device
    if not ports:
        return None
    if len(ports) == 1:
        return ports[0].device
    print("Several serial ports found:")
    for i, p in enumerate(ports, 1):
        print(f"  {i}. {p.device}  -  {p.description}")
    while True:
        choice = input("Choose the port number for the ESP32: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1].device


def read_exact(ser, n, deadline):
    data = b""
    while len(data) < n:
        if time.time() > deadline:
            raise TimeoutError("Timed out while receiving the image.")
        chunk = ser.read(n - len(data))
        if chunk:
            data += chunk
    return data


def ping(ser):
    ser.reset_input_buffer()
    ser.write(b"P")
    deadline = time.time() + 4
    buf = b""
    while time.time() < deadline:
        buf += ser.read(64)
        if b"PONG" in buf:
            line = buf[buf.index(b"PONG"):].split(b"\n")[0].decode(errors="ignore")
            return line.strip()
    return None


def request_jpeg(ser, cmd, timeout):
    """cmd = b'V' (quick preview frame) or b'C' (full photo, saved on the SD card)."""
    ser.reset_input_buffer()
    ser.write(cmd)
    deadline = time.time() + timeout
    window = b""
    while True:
        if time.time() > deadline:
            raise TimeoutError("The camera did not answer.")
        b = ser.read(1)
        if not b:
            continue
        window = (window + b)[-4:]
        if window == b"IMG!":
            break
        if window == b"ERR!":
            raise RuntimeError("The camera failed to take a picture.")
    length = struct.unpack("<I", read_exact(ser, 4, deadline))[0]
    if length == 0 or length > MAX_IMAGE_BYTES:
        raise RuntimeError(f"Bad image size received: {length}")
    return read_exact(ser, length, deadline)


def send_text(ser, title, body=""):
    """Show a text page on the ESP32's TFT screen (plain ASCII only)."""
    try:
        msg = (title + "\n" + body).encode("ascii", "replace")[:MAX_TFT_CHARS]
        ser.write(b"T" + struct.pack("<H", len(msg)) + msg)
        ser.flush()
    except Exception:
        pass  # the screen is a bonus; never stop the program because of it


def decode(jpeg):
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)


# ------------------------------- drawing helper -------------------------------
def overlay(img, lines, color=(0, 255, 0)):
    out = img.copy()
    y = 24
    for line in lines:
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    color, 1, cv2.LINE_AA)
        y += 24
    return out


def google_search(text):
    url = "https://www.google.com/search?q=" + quote_plus(text[:MAX_QUERY_LEN])
    print(f"Opening: {url}")
    webbrowser.open(url)


# ------------------------- capture, read, edit, search -------------------------
def capture_and_read(ser, reader):
    try:
        print("\nTaking photo...")
        jpeg = request_jpeg(ser, b"C", timeout=20)
    except (TimeoutError, RuntimeError) as e:
        print(f"Error: {e}")
        return

    img = decode(jpeg)
    if img is None:
        os.makedirs(SAVE_DIR, exist_ok=True)
        bad = os.path.join(SAVE_DIR, "bad_photo.jpg")
        with open(bad, "wb") as f:
            f.write(jpeg)
        print(f"The photo was damaged in transfer ({len(jpeg)} bytes, "
              f"starts {jpeg[:2].hex()}, ends {jpeg[-2:].hex()}). Please try again.")
        print(f"(The damaged data was saved as {bad} - send me the numbers above if it keeps happening.)")
        return

    os.makedirs(SAVE_DIR, exist_ok=True)
    name = datetime.now().strftime("capture_%Y%m%d_%H%M%S.jpg")
    path = os.path.join(SAVE_DIR, name)
    with open(path, "wb") as f:
        f.write(jpeg)
    print(f"Saved on laptop: {path}")
    print(">>> Click THIS terminal window now to answer the questions. <<<")

    time.sleep(0.6)  # let the TFT finish drawing the photo, then show a status page
    send_text(ser, "Photo captured", "Reading text...")

    state = {"done": False}

    def worker():
        try:
            print("Reading text...")
            lines = reader.readtext(img, detail=0, paragraph=True)
            text = " ".join(lines).strip()

            print("\n----- Text read from the photo -----")
            print(text if text else "(no text detected)")
            print("------------------------------------")
            if not text:
                send_text(ser, "No text found",
                          "Try better light, less glare, and keep the text in focus.")
                print("Try again with better light, less glare, and the text in focus.")
                return

            send_text(ser, "Text found (answer in terminal)", text)

            while True:
                if input("Do you want to edit/correct this text? (y/n): ").strip().lower() in ("y", "yes"):
                    new = input("Type the corrected text (ENTER keeps the current text):\n> ").strip()
                    if new:
                        text = new
                    print(f"\nText is now: {text}")
                    send_text(ser, "Corrected text (answer in terminal)", text)
                else:
                    break

            if input("Search this on Google? (y/n): ").strip().lower() in ("y", "yes", ""):
                send_text(ser, "Searching Google", text)
                google_search(text)
        finally:
            print("\nBack to live view. Click the video window and press SPACE for the next photo.\n")
            state["done"] = True

    threading.Thread(target=worker, daemon=True).start()

    # Keep the video window alive (showing the captured photo) while the terminal works.
    frozen = overlay(img, ["CAPTURED - answer the questions in the terminal"], (0, 200, 255))
    while not state["done"]:
        cv2.imshow(WINDOW, frozen)
        cv2.waitKey(50)


# --------------------------------- main app ---------------------------------
def run(port_arg, baud):
    load_cv()
    import serial

    print("=" * 56)
    print("  ESP32-S3 CAMERA  ->  OCR  ->  EDIT  ->  GOOGLE SEARCH")
    print("=" * 56)

    port = port_arg or find_port()
    if not port:
        print("No serial port found. Check the USB cable and try again.")
        return
    print(f"Connecting to {port} ...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("Close the Arduino Serial Monitor and try again.")
        return

    time.sleep(2.5)  # the board may restart when the port opens
    status = ping(ser)
    if status is None:
        print("The board did not answer. Is the sketch uploaded? Is the right port selected?")
        return
    print(f"Board connected ({status}).")
    if status.endswith("SD=0"):
        print("Note: no SD card detected, photos will only be saved on this laptop.")

    send_text(ser, "ESP32 OCR", "Loading the text reader on the laptop...")
    print("\nLoading EasyOCR (the first run downloads the model)...")
    import easyocr

    reader = easyocr.Reader(LANGUAGES, gpu=USE_GPU)

    # Check that the board understands the live-view command.
    try:
        request_jpeg(ser, b"V", timeout=8)
    except (TimeoutError, RuntimeError) as e:
        print(f"The board did not send a live frame ({e}).")
        print("Make sure the sketch esp32_camera_tft.ino is the one uploaded, Core Debug Level is")
        print("'None', and look at the Arduino Serial Monitor for 'CAM_ERR' or reboot messages.")
        return

    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    except cv2.error:
        print("\nYour OpenCV install cannot open video windows (the 'headless' version is installed).")
        print("Fix: close this program, run these two commands, then start it again:")
        print("  pip uninstall -y opencv-python opencv-python-headless")
        print("  pip install opencv-python")
        return
    clicked = {"flag": False}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked["flag"] = True

    cv2.setMouseCallback(WINDOW, on_mouse)

    print("\nLive view is open.")
    print("  -> Click the VIDEO window, then press SPACE (or click the video) to read text.")
    print("  -> Press Q in the video window to quit.\n")

    errors = 0
    fps = 0.0
    last = time.time()

    while True:
        try:
            jpeg = request_jpeg(ser, b"V", timeout=5)
            frame = decode(jpeg)
            if frame is None:
                raise RuntimeError("bad frame")
            errors = 0
        except (TimeoutError, RuntimeError):
            errors += 1
            if errors >= 10:
                print("Lost the camera. Check the USB cable and restart the program.")
                break
            continue

        now = time.time()
        inst = 1.0 / max(now - last, 1e-3)
        fps = inst if fps == 0 else 0.8 * fps + 0.2 * inst
        last = now

        if PREVIEW_SCALE != 1:
            frame = cv2.resize(frame, None, fx=PREVIEW_SCALE, fy=PREVIEW_SCALE,
                               interpolation=cv2.INTER_LINEAR)
        shown = overlay(frame, [f"LIVE  {fps:.1f} fps",
                                "SPACE / click = read text    Q = quit"])
        cv2.imshow(WINDOW, shown)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), ord("Q"), 27):
            break
        if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key in (32, 13) or clicked["flag"]:
            clicked["flag"] = False
            capture_and_read(ser, reader)

    send_text(ser, "ESP32 OCR", "Program closed on the laptop.")
    ser.close()
    cv2.destroyAllWindows()
    print("Goodbye.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="serial port, e.g. COM6 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--in-terminal", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    relaunch_in_terminal()  # exits here if it opened a new window

    try:
        run(args.port, args.baud)
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception:
        import traceback
        print("\n*** The program hit an error. Please send this text: ***\n")
        traceback.print_exc()
    finally:
        if args.in_terminal:
            input("\nPress ENTER to close this window...")


if __name__ == "__main__":
    main()
