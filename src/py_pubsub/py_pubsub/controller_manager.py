from pathlib import Path
from enum import Enum
from functools import partial
from numpy import clip
# Allow for this program to run standalone for testing without ROS packages
if __name__ != '__main__':
    from . import gamepad_input as gi
    DEBUG = True
else:
    DEBUG = True
    import gamepad_input as gi

class ControllerScheme(Enum):
    """Set of valid control schemas with their logic.
    
    Each scheme is a function accepting an input state and returning a vector
    to send to the rover.

    input_state: (ls_x, ls_y, rs_x, rs_y, lt, rt, hat_x, hat_y, a, b, x, y)
    """

    @staticmethod
    def _basic(*input_state) -> list[float]:
        """Basic input scheme internal logic
        
        Old control scheme from 2022-2023
        """
        ls_x, ls_y, rs_x, rs_y, lt, rt, hat_x, hat_y, a, b, x, y = input_state

        bicep_dir = rs_y
        forearm_dir = ls_y

        turret_dir = ls_x

        pitch_dir = hat_y
        roll_dir = hat_x

        # X close, O open
        grip_dir = 1.0 if a else -1.0 if b else 0

        # Triangle out, Square in
        poker_dir = 1.0 if y else -1.0 if x else 0

        # TODO: Handle extra motor

        # Pack message
        move_vec = [float(bicep_dir), float(forearm_dir), float(-turret_dir), float(-pitch_dir), float(roll_dir), 0.0, float(-grip_dir), float(poker_dir)]

        return move_vec


    BASIC = partial(_basic)

    def __call__(self, *args, **kwargs):
        """Allows for treating enumerated values as functions"""
        return self.value(*args, **kwargs)



class ControllerManager:
    """{DOCSTRING HERE}"""
    _scheme: ControllerScheme
    _gamepad_index: int
    _deadzone: float

    _stopped: bool
    _stop_pressed: bool
    _cruise_vec: (float, float)
    _cruise_pressed: bool

    def __init__(self, \
                 scheme: ControllerScheme = ControllerScheme.BASIC, \
                 gamepad_index: int = 0, \
                 deadzone: float = 0.1,
                 config_path: str = None) -> None:
        self._scheme = scheme

        self._configure_gamepad_input(config_path)
        self._gamepad_index = gamepad_index
        self._deadzone = deadzone

        self._stopped = False
        self._stop_pressed = False
        self._cruise_vec = None
        self._cruise_pressed = False

    def handle_input(self) -> list[float]:
        # Reinitialize gamepad each call to handle new connections
        gamepad = gi.getGamepad(self._gamepad_index)

        if not gamepad:
            print(f'WARN: No valid gamepad at {self._gamepad_index}!')
            return [0.0, 0.0]
    
        ls_x, ls_y = gi.getLeftStick(gamepad, self._deadzone)
        rs_x, rs_y = gi.getRightStick(gamepad, self._deadzone)
        # Negate all joystick values to obtain normal results
        ls_x, ls_y, rs_x, rs_y = -ls_x, -ls_y, -rs_x, -rs_y
        lt, rt = gi.getTriggers(gamepad, self._deadzone)
        hat_x, hat_y = gi.getHat(gamepad)

        plus, minus, home = gi.getButtonsValues(gamepad, 4, 5, 6)
        y, x, a, b = gi.getButtonsValues(gamepad, 0, 1, 2, 3)
        ls_b, rs_b = gi.getButtonsValues(gamepad, 9, 10)

        if DEBUG:
            print([i for i,b in \
               enumerate(gi.getButtonsValues(gamepad, *range(0, 18))) if b])

        # Emergency stop: press plus + minus to toggle
        if plus and minus:
            if not self._stop_pressed:
                self._stopped = not self._stopped
                self._stop_pressed = True
        elif self._stop_pressed:
            self._stop_pressed = False
        
        if self._stopped:
            self._cruise_vec = None
            return [0.0, 0.0]

        move_vec = self._scheme(ls_x, ls_y, rs_x, rs_y, lt, rt, hat_x, hat_y, \
                                a, b, x, y)
        
        # Cruise control: press home to toggle
        if home:
            if not self._cruise_pressed:
                self._cruise_vec = move_vec if not self._cruise_vec else None
                self._cruise_pressed = True
        elif self._cruise_pressed:
            self._cruise_pressed = False
        
        if self._cruise_vec:
            return self._cruise_vec
        
        return move_vec


    def change_scheme(self, new_scheme: ControllerScheme) -> None:
        if new_scheme == self._scheme:
            print("WARN: Attempted to change controller scheme to current!")
        else:
            self._scheme = new_scheme

    def _configure_gamepad_input(self, config_path: str) -> None:
        if not config_path:
            config_path = 'gamepads.config'
        config_path_str = str((Path(__file__).parent / config_path).resolve())
        
        gi.setConfigFile(config_path_str)
        gi.run_event_loop()


# For testing purposes
if __name__ == '__main__':
    from time import sleep
    import os
    manager = ControllerManager(config_path = \
                                'src/arm/config/gamepads.config')
    schemes_list = list(map(lambda x: x.name.lower(), ControllerScheme))
    test_scheme = input(f'Enter an input scheme to test {schemes_list}: ')
    if test_scheme.lower() in schemes_list:
        print(f'Testing {test_scheme} control scheme.')
    else:
        print('Error, invalid scheme name!')
        exit()

    manager.change_scheme(ControllerScheme[test_scheme.upper()])

    sleep(3)

    while True:
        os.system('cls||clear')
        print(f'{manager.handle_input()}')
        sleep(0.2)

