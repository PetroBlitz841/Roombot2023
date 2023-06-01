HUB_NAME = "PetroBlitz2"

import os
import asyncio
import subprocess

target = os.getenv("TARGET")
command = f"pybricksdev run ble --name {HUB_NAME} {target}"
try:
    subprocess.run(command, shell=True, check=True)
except subprocess.CalledProcessError as e:
    print("Failed to upload code to the hub. Did you set HUB_NAME?")
