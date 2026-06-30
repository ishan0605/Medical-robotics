import pyigtl
import cv2

def main():
    # Create an OpenIGTLink server
    server = pyigtl.OpenIGTLinkServer(port=18945)
    
    print("Waiting for connection...")
    server.start()

    try:
        print("Waiting for an image message...")
        while True:
            message = server.wait_for_message("HealsonProbe", timeout=5)  # Wait for any message
            
            if isinstance(message, pyigtl.ImageMessage):
                print(f"Received image: {message.device_name}")
                print(f"Image size: {message.image.shape}")
                
                # Save the received frame
                image = message.image
                if image.ndim == 2:  # If grayscale, convert to RGB
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
                elif image.ndim == 3 and image.shape[2] == 3:
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
                cv2.imwrite("received_frame.png", image)
                print("Saved received_frame.png")
                break  # Exit after saving one frame

    except Exception as e:
        print(f"Error: {e}")

    finally:
        print("Test completed. Exiting.")
        server.stop()

if __name__ == "__main__":
    main()
