import rospkg
from copy import copy
from PIL import Image, ImageDraw, ImageFont

rp = rospkg.RosPack()
pack_path = rp.get_path('brr_ui')
font = ImageFont.truetype('%s/share/FreeSerif.ttf' % pack_path, 25)

class BrrWindow():
    def __init__(self, window, buttons):
        self.name = window['name']
        self.bg = {}
        self.bg['normal'] = Image.open('%s/share/Panels/%s.png' % (pack_path, window['bg']))
        self.bg['offset'] = (window['offset'][0], window['offset'][1])
        self.bg['size'] = self.bg['normal'].size
        if window['bg'] == 'Panels_Main':
            self.bg['disabled'] = Image.open('%s/share/Panels/%s_DarkBkg.png' % (pack_path, window['bg']))
        else:
            self.bg['disabled'] = self.bg['normal']
        self.buttons = {} 
        for name, btn in buttons.items():
            if buttons[name].window == self.name:
                self.buttons[btn.index] = btn
        self.selected = window['default']
        self.parent = window['parent']
        if self.parent and len(self.buttons)> 1:
            self.selected_btn_index = 1
        else:
            self.selected_btn_index = 0
        
        self.states = {}
        self.states['normal'] = {}
        self.states['disabled'] = {}
        for id in self.buttons:
            name = self.buttons[id].name
            self.states['normal'][name] = self.gen_img(name, disabled=False)
            self.states['disabled'][name] = self.gen_img(name, disabled=True)
        
    def gen_img(self, sel_btn, disabled = False):
        if disabled: 
            tmp = copy(self.bg['disabled'])
        else: 
            tmp = copy(self.bg['normal'])
        d = ImageDraw.Draw(tmp)
        for name, btn in self.buttons.items():
            if btn.name == sel_btn:
                if disabled:
                    state = 'pressed'
                else:
                    state = 'selected'
            else:
                if disabled:
                    state = 'disabled'
                else:
                    state = 'idle'
            btn.draw(tmp, d, state)
        return tmp

    def draw(self, img, draw, selected=True):
        if selected:
            tmp = self.states['normal'][self.selected_btn().name]
        else:
            tmp = self.states['disabled'][self.selected_btn().name]
        img.paste(tmp, self.bg['offset'])
        return img

    def selected_btn(self):
        return self.buttons[self.selected_btn_index]
