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

from PIL import Image, ImageDraw, ImageFont
import rospkg


class BrrButton(object):
    def __init__(self, name, size, offset,
                 index, image_prefix, inner,
                 label='', selectable=True, share_path=''):
        self.name = name
        self.index = index
        self.selectable = selectable

        self._size = tuple(size)
        self._offset = tuple(offset)
        self._font = ImageFont.truetype('%s/FreeSerif.ttf' %
                                           share_path, 25)
        # "inner" refers to a group of image assets that only have 2 images)
        if inner:
            base_path = ('%s/Buttons/Buttons_Inner_%s' %
                         (share_path, image_prefix))
        else:
            base_path = ('%s/Buttons/Buttons_%s' %
                         (share_path, image_prefix))
        self._imgs = dict()
        self._imgs['idle'] = Image.open('%s.png' % base_path)
        self._imgs['selected'] = Image.open('%s_Pressed.png' % base_path)
        if inner:
            self._imgs['disabled'] = self._imgs['idle']
            self._imgs['pressed'] = self._imgs['selected']
        else:
            self._imgs['disabled'] = Image.open('%s_Dis.png' % base_path)
            self._imgs['pressed'] = Image.open('%s_ON.png' % base_path)
        self._label = label

    def get_image(self, state):
        return self._imgs[state].resize(self._size, Image.ANTIALIAS)

    def draw_label(self, draw):
        label_x = (self._offset[0] + self._size[0] / 2 -
                   draw.textsize(self._label, self._font)[0] / 2)
        label_y = self._offset[1] + self._size[1]
        draw.text((label_x, label_y), self._label,
                  fill='white', font=self._font)

    def draw(self, img, draw, state):
        tmp = self.get_image(state)
        self.draw_label(draw)
        img.paste(tmp, self._offset)
