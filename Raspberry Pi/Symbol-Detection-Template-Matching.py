import cv2
import numpy as np
import os
import time
from picamera2 import Picamera2

def load_templates_from_subfolders(subfolder_paths):
    """
    Load template images from a list of subfolder paths.
    Each subfolder's name is used as the label.

    :param subfolder_paths: List of paths to subfolders containing template images.
    :return: Dictionary where keys are subfolder names (labels) and values are lists of grayscale template images.
    """
    templates = {}
    for subfolder in subfolder_paths:
        if os.path.isdir(subfolder):
            label = os.path.basename(subfolder)
            templates[label] = []
            for filename in os.listdir(subfolder):
                if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
                    img_path = os.path.join(subfolder, filename)
                    template = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                    if template is not None:
                        templates[label].append(template)
        else:
            print(f"Subfolder path not found: {subfolder}")
    return templates

def match_templates(frame, templates, threshold=0.6, scales=[0.5, 0.75, 1.0, 1.25, 1.5]):
    """
    Perform multi-scale template matching on the frame using all templates from each label.

    :param frame: Input image (expects a BGR image for consistency).
    :param templates: Dictionary of templates loaded from subfolders.
    :param threshold: Minimum matching score required.
    :param scales: List of scales to try for each template.
    :return: Tuple (label, location, template_shape, score, scale) for the best match; None if no match.
    """
    # Create a grayscale copy for matching
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    best_label = None
    best_location = None
    best_template_shape = None
    best_score = 0.0
    best_scale = None

    # Initialize cache for resized templates if not already done
    if not hasattr(match_templates, 'cache'):
        match_templates.cache = {}

    for label, template_list in templates.items():
        for template in template_list:
            for scale in scales:
                # Use caching to avoid resizing the same template repeatedly
                cache_key = (id(template), scale)
                if cache_key in match_templates.cache:
                    resized_template = match_templates.cache[cache_key]
                else:
                    resized_template = cv2.resize(template, None, fx=scale, fy=scale)
                    match_templates.cache[cache_key] = resized_template

                # Skip if resized template is too large for the frame
                if resized_template.shape[0] >= gray.shape[0] or resized_template.shape[1] >= gray.shape[1]:
                    continue
                res = cv2.matchTemplate(gray, resized_template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(res)
                if max_val > best_score and max_val >= threshold:
                    best_score = max_val
                    best_label = label
                    best_location = max_loc
                    best_template_shape = resized_template.shape  # (height, width)
                    best_scale = scale

    if best_label is not None:
        return best_label, best_location, best_template_shape, best_score, best_scale
    else:
        return None

def main():
    # List your subfolder paths here.
    subfolder_paths = [
        "/home/pi/ref-frm-picam/back",
        "/home/pi/ref-frm-picam/circle",
        "/home/pi/ref-frm-picam/dist",
        "/home/pi/ref-frm-picam/face",
        "/home/pi/ref-frm-picam/fwd",
        "/home/pi/ref-frm-picam/hexagon",
        "/home/pi/ref-frm-picam/left",
        "/home/pi/ref-frm-picam/partial-circle",
        "/home/pi/ref-frm-picam/pentagon",
        "/home/pi/ref-frm-picam/rectangle",
        "/home/pi/ref-frm-picam/right",
        "/home/pi/ref-frm-picam/stop",
        "/home/pi/ref-frm-picam/traffic",
        "/home/pi/ref-frm-picam/triangle"
    ]
    
    templates = load_templates_from_subfolders(subfolder_paths)
    
    # Initialize PiCamera2 with desired resolution and format.
    picam2 = Picamera2()
    # Set resolution and ensure the output format is RGB (which is how the sensor sees colors)
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.configure("preview")
    picam2.start()
    
    try:
        while True:
            # Capture a frame (this frame is in RGB).
            rgb_frame = picam2.capture_array()
            # Make a copy for matching (we convert the copy to BGR for display later).
            # Here, we perform matching on a temporary BGR copy.
            frame_for_match = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            
            # Attempt to match a symbol using multi-scale template matching.
            result = match_templates(frame_for_match, templates, threshold=0.6)
            if result is not None:
                label, loc, tmpl_shape, score, scale = result
                top_left = loc
                bottom_right = (top_left[0] + tmpl_shape[1], top_left[1] + tmpl_shape[0])
                cv2.rectangle(frame_for_match, top_left, bottom_right, (0, 255, 0), 2)
                cv2.putText(frame_for_match, f"{label} ({score:.2f})", (top_left[0], top_left[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                print('Result is none')

            # Now convert back to RGB for display so that the colors are as seen in the real world.
            display_frame = cv2.cvtColor(frame_for_match, cv2.COLOR_BGR2RGB)
            cv2.imshow("Symbol Recognition", display_frame)
            
            # Exit if 'q' is pressed.
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            
            time.sleep(0.1)
    
    except Exception as e:
        print("An error occurred:", e)
    
    finally:
        cv2.destroyAllWindows()
        picam2.stop()
        
if __name__ == "__main__":
    main()
