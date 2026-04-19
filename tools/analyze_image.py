import subprocess
import time
import sys
from pathlib import Path

def analyze_image_with_ollama(image_path, prompt, model="llava:7b"):
    # Prepare the command for ollama interactive mode with image
    # Use echo with newline to simulate interactive input
    cmd = [
        "ollama", "run", model
    ]
    # The prompt and image path are sent as input
    input_text = f"{prompt}\n{image_path}\n"
    start = time.time()
    result = subprocess.run(cmd, input=input_text.encode(), capture_output=True, timeout=120)
    elapsed = time.time() - start
    return result.stdout.decode(), elapsed

if __name__ == "__main__":
    # Example usage: python3 analyze_image.py /path/to/image.jpg
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_image.py /path/to/image.jpg")
        sys.exit(1)
    image_path = Path(sys.argv[1]).resolve()
    if not image_path.exists():
        print(f"Image not found: {image_path}")
        sys.exit(1)
    prompt = "This is a greyscale night vision image. Is there a cat in this image? If yes, is it carrying prey?"
    print(f"Analyzing {image_path} with {prompt}")
    output, elapsed = analyze_image_with_ollama(str(image_path), prompt)
    print("--- Model Output ---")
    print(output)
    print(f"--- Time taken: {elapsed:.2f} seconds ---")
