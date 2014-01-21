import rospkg
from PIL import Image, ImageDraw, ImageFont

rp = rospkg.RosPack()
pack_path = rp.get_path('brr_ui')
font = ImageFont.truetype('%s/share/FreeSerif.ttf' % pack_path, 25)

class BrrButton():
    def __init__(self, name, size, offset, index, image_prefix, inner, window, label='', selectable=True):
        self.name = name
        self.size = (size[0], size[1])
        self.offset = (offset[0], offset[1])
        self.index = index
        if inner:
            base_path = '%s/share/Buttons/Buttons_Inner_%s' % (pack_path, image_prefix)
        else:
            base_path = '%s/share/Buttons/Buttons_%s' % (pack_path, image_prefix)

        self.idle_img = Image.open('%s.png' % base_path)
        self.selected_img = Image.open('%s_Pressed.png' % base_path)
        if inner:
            self.disabled_img = self.idle_img
            self.pressed_img = self.selected_img
        else:
            self.disabled_img = Image.open('%s_Dis.png' % base_path)
            self.pressed_img = Image.open('%s_ON.png' % base_path)
        self.window = window
        self.label = label
        self.selectable = selectable

    def get(self, state):
        #print '--@button get(): name: %s   state: %s' % (self.name, state)
        if state == 'idle':
            tmp = self.idle_img
        elif state == 'selected':
            tmp = self.selected_img
        elif state == 'pressed':
            tmp = self.pressed_img
        else:
            tmp = self.disabled_img
        return tmp.resize(self.size, Image.ANTIALIAS)

    def draw_label(self, draw):
        label_x = self.offset[0] + self.size[0]/2 - draw.textsize(self.label, font=font)[0]/2
        label_y = self.offset[1] + self.size[1]
        draw.text((label_x, label_y), self.label, fill='white', font = font)
    
    def draw(self, img, draw, state):
        tmp = self.get(state)
        self.draw_label(draw)
        img.paste(tmp, self.offset)

