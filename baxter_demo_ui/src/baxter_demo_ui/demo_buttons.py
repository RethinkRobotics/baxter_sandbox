from PIL import Image, ImageDraw, ImageFont
import rospkg


class BrrButton(object):
    def __init__(self, name, size, offset,
                 index, image_prefix, inner,
                 label='', selectable=True, share_path=''):
        self.name = name
       
        self.size = tuple(size)
        self.share_path = share_path
        self.font = ImageFont.truetype('%s/FreeSerif.ttf' %
                                           self.share_path, 25)
        self.offset = tuple(offset)
        self.index = index
        # "inner" refers to a group of image assets that only have 2 images)
        if inner:
            base_path = ('%s/Buttons/Buttons_Inner_%s' %
                         (self.share_path, image_prefix))
        else:
            base_path = ('%s/Buttons/Buttons_%s' %
                         (self.share_path, image_prefix))

        self.idle_img = Image.open('%s.png' % base_path)
        self.selected_img = Image.open('%s_Pressed.png' % base_path)
        if inner:
            self.disabled_img = self.idle_img
            self.pressed_img = self.selected_img
        else:
            self.disabled_img = Image.open('%s_Dis.png' % base_path)
            self.pressed_img = Image.open('%s_ON.png' % base_path)
        self.label = label
        self.selectable = selectable

    def get_image(self, state):
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
        label_x = (self.offset[0] + self.size[0]/2 -
                   draw.textsize(self.label, self.font)[0]/2)
        label_y = self.offset[1] + self.size[1]
        draw.text((label_x, label_y), self.label, fill='white', font = self.font)

    def draw(self, img, draw, state):
        tmp = self.get_image(state)
        self.draw_label(draw)
        img.paste(tmp, self.offset)

