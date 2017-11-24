import io
import os
import time
from google.cloud import vision
from google.cloud.vision import types

class GCPvision:

    def __init__(self):
        self.gcpClient = vision.ImageAnnotatorClient()
        self.file_name = ""

    def setFile(self, fname):
        self.file_name = fname

    def getLabels(self):
        image_file = io.open(self.file_name, 'rb')
        content = image_file.read()
        image = types.Image(content=content)
        response = self.gcpClient.label_detection(image=image)
        labels = response.label_annotations
        print('Labels:')
        for label in labels:
            print("mid: %s desc: %s score %1.3f" % (label.mid, label.description, label.score))


if __name__ == '__main__':
    print("gcp_vision test starting")
    stime = time.time()
    gcpVision = GCPvision()
    gcpVision.setFile('/opt/robot/img/weed/roboimg1502012231.png')
    gcpVision.getLabels()
    etime = time.time()
    difftime = etime - stime
    print("gcp_vision test end in %d ticks" % (difftime))
    
        
        
