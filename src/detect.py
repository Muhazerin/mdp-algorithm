import ssl

import torch
import operator
import os
import numpy as np
import cv2

SYMBOL_ID_MAP = {
    'six': 6,
    'seven': 7,
    'eight': 8,
    'nine': 9,
    'zero': 10,
    'redV': 11,
    'greenW': 12,
    'whiteX': 13,
    'blueY': 14,
    'yellowZ': 15,
    'rightArrow': 3,
    'greenCircle': 5,
    'leftArrow': 4,
    'downArrow': 2,
    'upArrow': 1
}

img_list = [
    [0, 0],
    [0, 0],
    [0, 0],
    [0, 0],
    [0, 0]
]

file_name = 'Z:/1.png'


def xyxy2xywh(x):
    # Convert nx4 boxes from [x1, y1, x2, y2] to [x, y, w, h] where xy1=top-left, xy2=bottom-right
    y = torch.zeros_like(x) if isinstance(x, torch.Tensor) else np.zeros_like(x)
    y[:, 0] = (x[:, 0] + x[:, 2]) / 2  # x center
    y[:, 1] = (x[:, 1] + x[:, 3]) / 2  # y center
    y[:, 2] = x[:, 2] - x[:, 0]  # width
    y[:, 3] = x[:, 3] - x[:, 1]  # height
    return y


def get_result(results, gn):
    predict = {}
    for i, (img, pred) in enumerate(zip(results.imgs, results.pred)):
        if pred is not None:
            for _ in pred[:, -1].unique():
                conf: float
                for *box, conf, cls in pred:    #xyxy, confidence, class
                    try:
                        xywh = (xyxy2xywh(torch.tensor(box).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        # print(f'x: {round(xywh[0], 3)}, y: {round(xywh[1], 3)}, w: {round(xywh[2], 3)}, h: '
                        #       f'{round(xywh[3], 3)}, name: {results.names[int(cls)]}')
                        predict[results.names[int(cls)]] = [float(conf), round(xywh[0], 3), round(xywh[1], 3),
                                                            round(xywh[2], 3), round(xywh[3], 3)]
                    except:
                        continue
        else:
            break
    return predict


def get_array_from_list(img_id):
    for item in img_list:
        if item[0] == img_id:
            return item
    for item in img_list:
        if item[0] == 0:
            return item


def get_prediction(model):
    prediction = []
    try:
        print('[Detection Algo] Detecting image')
        image = cv2.imread(file_name)[:, :, ::-1]
        results = model(image)
        gn = torch.tensor(image.shape)[[1, 0, 1, 0]]
        # results.show()

        predict = get_result(results, gn)
        print(f'Detection: {predict}')
        if predict:
            temp_predict = {}
            for item in predict:
                if predict[item][3] >= 0.18 and predict[item][4] >= 0.18:   # if the width and height >= 0.2
                    # append to another dictionary
                    temp_predict[item] = predict[item]

            if temp_predict:
                class_predict = max(temp_predict.items(), key=operator.itemgetter(1))[0]
                class_items = max(temp_predict.items(), key=operator.itemgetter(1))[1]
                print(f'Highest confidence: {class_predict}, {class_items[0]}, {class_items[1]}')
                if 0.00 <= class_items[1] < 0.33:
                    # image on left
                    prediction = [SYMBOL_ID_MAP[class_predict], 1]
                elif 0.33 <= class_items[1] < 0.66:
                    # image on middle
                    prediction = [SYMBOL_ID_MAP[class_predict], 0]
                elif 0.66 <= class_items[1] <= 1.00:
                    # image of right
                    prediction = [SYMBOL_ID_MAP[class_predict], -1]
                item = get_array_from_list(SYMBOL_ID_MAP[class_predict])
                if item == [0, 0]:
                    item[0] = SYMBOL_ID_MAP[class_predict]
                    item[1] = class_items[0]
                    results.save(f'images/{SYMBOL_ID_MAP[class_predict]}')
                else:
                    if item[1] < class_items[0]:
                        item[1] = class_items[0]
                        results.save(f'images/{SYMBOL_ID_MAP[class_predict]}')
            else:
                print(f'[Detection] Image detected but they are far away')

        print('[Algo - Detect] Removing the photo from shared folder')
        os.remove(file_name)       # Delete the file as I faced the error of getting the same photo at next take photo

        return prediction
    except Exception as err:
        print('[Detect] Error')
        return []


if __name__ == '__main__':
    ssl._create_default_https_context = ssl._create_unverified_context

    model = torch.hub.load('ultralytics/yolov5', 'custom', path_or_model='best.pt')

    print(get_prediction(model))
