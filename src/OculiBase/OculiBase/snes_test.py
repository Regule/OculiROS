import rclpy
import serial
import pygame as pg
from rclpy.node import Node
from serial.tools import list_ports 

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


def select_board():
    ports = list(map(str,list_ports.comports()))
    if len(ports) == 0:
        print('No serial ports avaialble.')
        return None
    print('Ports available :')
    for choice, port in enumerate(ports):
        print(f'{choice} -> {port}')
    choice = int(input('Select port >'))
    try:
        port = ports[choice].split()[0]
        baudrate = 9600
        timeout = 0.3
        board = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        return board
    except IndexError:
        print(f'No port at number {choice}')
        return None


class SnesController(Node):

    def __init__(self, node_name, controller, board, update_period):
        super().__init__(node_name)
        self.controller = controller
        self.board = board
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
            pwm_left = abs(int(255*axes[0]))
            pwm_right = abs(int(255*axes[1]))
            dir_left = 0 if axes[0]>=0 else 1
            dir_right= 0 if axes[1]>=0 else 1
            self.board.write(bytes(f'>{dir_left}#{pwm_left}#{dir_right}#{pwm_right}','ASCII'))
            self.board.write(bytes('<','ASCII'))
            response = self.board.readline().decode('ASCII')
            self.get_logger().info(f'BOARD ---> {response}')
        else:
            print('Controller not initialized.')

def main(args=None):
    pg.init()
    controller = select_controller()
    board = select_board()
    if controller is None or board is None:
        return
    rclpy.init(args=args)
    snes = SnesController('snes', controller, board, 0.5)
    rclpy.spin(snes)
    snes.destroy_node()


if __name__ == '__main__':
    main()
