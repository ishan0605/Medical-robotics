import pyautogui
import cv2
import numpy as np
import time
import pygetwindow as gw
import pyigtl

def switch_to_program(program_name):
    try:
        window = gw.getWindowsWithTitle(program_name)[0]
        window.activate()
        time.sleep(1)  # Wait for the window to become active
    except IndexError:
        print(f"Could not find window with title '{program_name}'")

def capture_area(x, y, width, height):
    screenshot = pyautogui.screenshot(region=(x, y, width, height))
    return cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)

def save_video(filename, frame):
    if not hasattr(save_video, "writer"):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        save_video.writer = cv2.VideoWriter(filename, fourcc, 30.0, (frame.shape[1], frame.shape[0]))
    save_video.writer.write(frame)

def stream_over_openigtlink(frame, client):
    image_message = pyigtl.ImageMessage(frame, device_name="HealsonProbe")
    client.send_message(image_message)

def main():
    program_name = "UProbeMain"
    capture_area_coords = (200, 200, 3000, 2000)  # Example coordinates (x, y, width, height)
    output_filename = "captured_video.mp4"
    
    # Switch to UProbeMain
    switch_to_program(program_name)
    
    # Set up OpenIGTLink client
    client = pyigtl.OpenIGTLinkClient(host="127.0.0.1", port=18945)
    
    try:
        while True:
            frame = capture_area(*capture_area_coords)
            save_video(output_filename, frame)
            stream_over_openigtlink(frame, client)
            time.sleep(0.033)  # Adjust for desired frame rate (30 fps in this case)
    except KeyboardInterrupt:
        print("Capturing stopped")
    finally:
        if hasattr(save_video, "writer"):
            save_video.writer.release()
        client.disconnect()

if __name__ == "__main__":
    main()
