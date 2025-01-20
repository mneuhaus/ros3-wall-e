import os
import subprocess
import sys
import time

def upload_main_py(main_py_path):
    """Upload the main.py file using rshell."""
    max_attempts = 3
    for attempt in range(max_attempts):
        try:
            print(f"Attempting to upload {main_py_path} (attempt {attempt + 1}/{max_attempts})...")
            subprocess.run(['rshell', 'cp', main_py_path, '/pyboard/main.py'], check=True)
            subprocess.run(['rshell', 'repl ~ import machine ~ machine.soft_reset() ~'], check=True)
            print("main.py uploaded successfully!")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Attempt {attempt + 1} failed: {e}")
            if attempt < max_attempts - 1:
                print("Retrying in 2 seconds...")
                time.sleep(2)
            else:
                print("Failed to upload main.py after multiple attempts")
                return False

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    main_py_path = os.path.join(script_dir, '../firmware/main.py')

    try:
        print("\nUploading main.py to device...")
        if not upload_main_py(main_py_path):
            sys.exit(1)

        print("\nAll done! Your code has been uploaded.")
        print("The board will automatically restart and run the new code.")

    except Exception as e:
        print(f"Error during upload: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
