from copy import copy
from PIL import Image, ImageDraw, ImageFont
import rospkg


class BrrWindow(object):
    def __init__(self, window_data, buttons, share_path):
        self.share_path = share_path
        self.font = ImageFont.truetype('%s/FreeSerif.ttf' %
                                           self.share_path, 25)
        self.name = window_data['name']
        self.bg = dict()
        self.bg['normal'] = Image.open('%s/Panels/%s.png' %
                                           (self.share_path,
                                            window_data['bg']))
        self.bg['offset'] = (window_data['offset'][0],
                             window_data['offset'][1])
        self.bg['size'] = self.bg['normal'].size
        self.no_scroll = window_data['no_scroll'] 
        if window_data['bg'] == 'Panels_Main':
            self.bg['disabled'] = Image.open('%s/Panels/'
                                                 '%s_DarkBkg.png' %
                                                 (self.share_path, 
                                                  window_data['bg']))
        else:
            self.bg['disabled'] = self.bg['normal']
        self.buttons = {} 
        for name, btn in buttons.items():
            self.buttons[btn.index] = btn
        self.selected = window_data['default']
        self.parent = window_data['parent']
        if self.parent and len(self.buttons) > 1:
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

    def draw(self, img, selected=True):
        if selected:
            tmp = self.states['normal'][self.selected_btn().name]
        else:
            tmp = self.states['disabled'][self.selected_btn().name]
        img.paste(tmp, self.bg['offset'])
        return img

    def selected_btn(self):
        return self.buttons[self.selected_btn_index]
