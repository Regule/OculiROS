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


class SnesController(Node):

    def __init__(self, node_name, controller, update_period):
        super().__init__(node_name)
        self.controller = controller
        self.timer = self.create_timer(update_period, self.update)

    def update(self):
        for event in pg.event.get():
            if event.type == pg.JOYBUTTONDOWN:
                self.get_logger().info("Joystick button pressed.")
            if event.type == pg.JOYBUTTONUP:
                self.get_logger().info("Joystick button released.")

        if self.controller.get_init():
            axes = [self.controller.get_axis(a) for a in range(self.controller.get_numaxes())]
            buttons = [self.controller.get_button(b) for b in range(self.controller.get_numbuttons())]
            self.get_logger().info(f'JOY ---> axes-{axes} buttons-{buttons}')
        else:
            print('Controller not initialized.')

def main(args=None):
    pg.init()
    controller = select_controller()
    if controller is None:
        return
    rclpy.init(args=args)
    snes = SnesController('snes', controller, 0.5)
    rclpy.spin(snes)
    snes.destroy_node()


if __name__ == '__main__':
    main()
