#!/usr/bin/env python

# Copyright (c) 2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from copy import copy
from PIL import Image, ImageDraw, ImageFont
import rospkg


class BrrWindow(object):
    def __init__(self, window_data, buttons, share_path):
        self.name = window_data['name']
        self.parent = window_data['parent']

        self._no_scroll = window_data['no_scroll']
        self._selected = window_data['default']

        self._font = ImageFont.truetype('%s/FreeSerif.ttf' %
                                           share_path, 25)
        self._bg = dict()
        self._bg['normal'] = Image.open('%s/Panels/%s.png' %
                                           (share_path,
                                            window_data['bg']))
        self._bg['offset'] = (window_data['offset'][0],
                             window_data['offset'][1])
        self._bg['size'] = self._bg['normal'].size
        if window_data['bg'] == 'Panels_Main':
            self._bg['disabled'] = Image.open('%s/Panels/'
                                                 '%s_DarkBkg.png' %
                                                  (share_path,
                                                   window_data['bg']))
        else:
            self._bg['disabled'] = self._bg['normal']
        self._buttons = {}
        for name, btn in buttons.items():
            self._buttons[btn.index] = btn
        print buttons
        print self.parent, len(self._buttons), self.name
        if self.parent and len(self._buttons) > 1:
            self._selected_btn_index = 1
        else:
            self._selected_btn_index = 0

        self._states = {}
        self._states['normal'] = {}
        self._states['disabled'] = {}
        for id in self._buttons:
            name = self._buttons[id].name
            self._states['normal'][name] = self._gen_img(name, disabled=False)
            self._states['disabled'][name] = self._gen_img(name, disabled=True)

    def draw(self, img, selected=True):
        if selected:
            tmp = self._states['normal'][self.selected_btn().name]
        else:
            tmp = self._states['disabled'][self.selected_btn().name]
        img.paste(tmp, self._bg['offset'])
        return img

    def selected_btn(self):
        print self.parent
        return self._buttons[self._selected_btn_index]

    def set_btn_selectable(self, index, sel):
        self._buttons[i].selectable = True

    def scroll(self, direction):
        print '--@scroll():  direction=%s' % direction
        if not self._no_scroll:
            i = self._selected_btn_index + direction
            while (i >= 0 and i < len(self._buttons)):
                if self._buttons[i].selectable:
                    self._selected_btn_index = i
                    break
                i += direction

    def _gen_img(self, sel_btn, disabled=False):
        if disabled:
            tmp = copy(self._bg['disabled'])
        else:
            tmp = copy(self._bg['normal'])
        d = ImageDraw.Draw(tmp)
        for name, btn in self._buttons.items():
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
