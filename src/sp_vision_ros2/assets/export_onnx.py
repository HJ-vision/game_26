# -*- coding: utf-8 -*-
from openvino import convert_model
from openvino.runtime import serialize

model = convert_model('yolov5.xml')
serialize(model, 'yolov5.xml', 'yolov5.bin')
print('model loaded')