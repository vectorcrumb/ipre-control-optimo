import pyglet
from pyglet import shapes
from pyglet.gl import glClearColor


window = pyglet.window.Window(900, 900, resizable=False, caption='Simulador Brazo 2DOF')
glClearColor(0.933, 0.933, 0.894, 1.0)

xw_0 = window.width // 2
yw_0 = window.height // 2


def coord_trans(xr, yr):
    return xr + xw_0, yr + yw_0

def xw_trans(xr):
    return xr + xw_0

def yw_trans(yr):
    return yr + yw_0




class RobotArm2DOF:
    joint_bold_color = (226, 135, 67)
    joint_color = (234, 182, 118)
    link_bold_color = (30, 129, 176)
    link_color = (118, 181, 197)
    joint_radius = 20
    link_length = 200
    link_height = 15

    def __init__(self):
        self.arm_batch = pyglet.graphics.Batch()
        self.base_link = pyglet.shapes.Rectangle(x=xw_trans(0), y=yw_trans(0), width=self.link_length,
                                                 height=self.link_height, color=self.link_color, batch=self.arm_batch)
        self.end_link = pyglet.shapes.Rectangle(x=xw_trans(self.link_length), y=yw_trans(0), width=self.link_length,
                                                height=self.link_height, color=self.link_color, batch=self.arm_batch)
        self.base_joint = pyglet.shapes.Circle(x=xw_trans(0), y=yw_trans(0), radius=self.joint_radius,
                                          color=self.joint_color, batch=self.arm_batch)
        self.mid_joint = pyglet.shapes.Circle(x=xw_trans(self.link_length), y=yw_trans(0), radius=self.joint_radius,
                                         color=self.joint_color, batch=self.arm_batch)
        self.end_joint = pyglet.shapes.Circle(x=xw_trans(2 * self.link_length), y=yw_trans(0), radius=self.joint_radius,
                                         color=self.joint_color, batch=self.arm_batch)


    def get_batch(self):
        return self.arm_batch

arm = RobotArm2DOF()

label = pyglet.text.Label('Hello World', font_name='Times New Roman', font_size=36,
                          x=window.width // 2, y=window.height // 2, anchor_x='center', anchor_y='center', color=(0, 255, 0, 255))

@window.event
def on_draw():
    window.clear()
    # label.draw()
    # arm_batch.draw()
    arm.get_batch().draw()


pyglet.app.run()