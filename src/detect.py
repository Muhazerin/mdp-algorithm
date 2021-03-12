import cv2
import torch
from PIL import Image
import ssl
import operator


def get_prediction():
    ssl._create_default_https_context = ssl._create_unverified_context

    model = torch.hub.load('ultralytics/yolov5', 'custom', path_or_model='last.pt')

    image = Image.open('Z:/1.png')
    # image = Image.open('sixtest.png')
    results = model(image)
    results.show()


    def get_result(results):
        predict = {}
        for i, (img, pred) in enumerate(zip(results.imgs, results.pred)):
            if pred is not None:
                for _ in pred[:, -1].unique():
                    conf: float
                    for *box, conf, cls in pred:  # xyxy, confidence, class
                        try:
                            predict[results.names[int(cls)]] = float(conf)
                        except:
                            continue
                        # label = f'{results.names[int(cls)]} {conf:.2f}'

            else:
                break
        return predict


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


    predict = get_result(results)
    print(predict)
    class_predict = max(predict.items(), key=operator.itemgetter(1))[0]
    print(class_predict)

    return SYMBOL_ID_MAP[class_predict]


if __name__ == '__main__':
    get_prediction()
