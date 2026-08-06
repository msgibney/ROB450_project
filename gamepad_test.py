from gamepad import Gamepad

import logging
import sys

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


logger.info("Setting up connections")
gamepad = Gamepad()

while True:
    gamepad.read_gamepad()
    logger.info(f"{gamepad.get_analogL_x()} {gamepad.get_analogL_y()}")