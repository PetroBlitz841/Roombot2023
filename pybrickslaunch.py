HUB_NAME = "Pybricks Hub"

import os
import subprocess

target = os.getenv("TARGET")
command = f"pybricksdev run ble --name {HUB_NAME} {target}"
subprocess.run(command, shell=True, check=True)