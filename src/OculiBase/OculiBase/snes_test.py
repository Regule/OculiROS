import rclpy
import pygame as pg
from rclpy.node import Node


def select_controller():
    pg.joystick.init()
    controllers = [pg.joystick.Joystick(x) for x in range(pg.joystick.get_count())]
    if len(controllers) == 0:
        print('No controller connected.')
        return None
    for idx, controller in enumerate(controllers):
        print(f'{idx} ---> {controller.get_name()}')
    choice = int(input('Select controller >'))
    try:
        controllers[choice].init()
        return controllers[choice]
    except IndexError:
        print(f'No controller at id {choice}')
        return None


class SnesController:

    def __init__(self, node_name, controller, update_period):
        super().__init__(node_name)
        self.controller = controller
        self.timer = self.create_timer(update_period, self.update)

    def update():
        pass


def main(args=None):
    controller = select_controller()
    if controller is None:
        return


if __name__ == '__main__':
    main()
