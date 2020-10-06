#!/usr/bin/env python
import torch
import torch.nn as nn
import torchvision
import torch.nn.functional as F
import torchvision.transforms as transforms
import torchvision.models as models

import numpy as np
from PIL import Image
import os
import rospy
from dslam_sp.msg import image_depth, PRrepresentor
from sensor_msgs.msg import Image as Image_msg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys, getopt

bridge = None
net = None
transform = None
representor_pub = None


def match(vecs, qvecs, threshold=0.95):
    # return matching image index if succeed;
    # return -1 if fail
    scores = np.dot(vecs.T, qvecs)
    ranks = np.argsort(-scores, axis=0)
    max_scores = np.max(scores, axis=0)
    best_match = ranks[0,:]
    best_match[max_scores < threshold] = -1
    return best_match


def extract_vectors(net, images, transform):
    net.cuda()
    vecs = np.zeros((net.meta['outputdim'], len(images)))
    for i in range(len(images)):
        fn = images[i]
        with open(fn, 'rb') as f:
            img = Image.open(f)
            img = img.convert('RGB')
        v = net(transform(img).unsqueeze(0).cuda())
        vecs[:,i] = v.cpu().data.squeeze()
        if (i+1) % 10 == 0 or (i+1) == len(images):
            print('\r>>>> {}/{} done...'.format((i+1), len(images)))
    return vecs


def extract_vectors_np(net, images, transform):
    net.cuda()
    if len(images.shape) > 2:
        if images.shape[2] == 1:
            image3c = cv2.cvtColor(images, cv2.COLOR_GRAY2BGR)
        else:
            image3c = images
    elif len(images.shape) == 2:
        image3c = cv2.cvtColor(images, cv2.COLOR_GRAY2BGR)
    try:
        img = Image.fromarray(image3c)
    except Exception as e:
        print("image3c")
        print(e)
        print(image3c)
        return False, None
    v = net(transform(img).unsqueeze(0).cuda())
    vecs = v.cpu().data.squeeze()
    return True, vecs

class GeM(nn.Module):
    def __init__(self, eps=1e-6):
        super(GeM, self).__init__()
        self.eps = eps

    def forward(self, x):
        return F.avg_pool2d(x.clamp(min=self.eps).pow(3), (x.size(-2), x.size(-1))).pow(1./3)
    
class L2N(nn.Module):
    def __init__(self, eps=1e-6):
        super(L2N,self).__init__()
        self.eps = eps

    def forward(self, x):
        return x / (torch.norm(x, p=2, dim=1, keepdim=True) + self.eps).expand_as(x)
    
class ImageRetrievalNet(nn.Module):
    def __init__(self):
        super(ImageRetrievalNet, self).__init__()
        net_in = getattr(torchvision.models, 'resnet101')(pretrained=False)
        features = list(net_in.children())[:-2]
        self.features = nn.Sequential(*features)
        self.pool = GeM()
        self.norm = L2N()
        meta = {'outputdim': 2048}
        self.meta = meta
    def forward(self, x):
        # x -> features
        o = self.features(x)
        
        # features -> pool -> norm
        o = self.norm(self.pool(o)).squeeze(-1).squeeze(-1)
        # permute so that it is Dx1 column vector per image (DxN if many images)
        return o.permute(1,0)

def main(argv):
    
    # load model
    global  bridge,net,transform,representor_pub
    
    gemweightspath = "/path/to/gem/path"
    opts, args = getopt.getopt(argv,"w:")
    for opt, arg in opts:
        if opt in ("-w"):
            gemweightspath = arg
    
    net = ImageRetrievalNet()
    state = torch.load(gemweightspath)
    net.load_state_dict(state['state_dict'], strict=False)
    
    # image preprocess
    normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    )
    transform = transforms.Compose([
        transforms.Resize(256),
        transforms.ToTensor(),
        normalize
    ])
    
    bridge = CvBridge()
    rospy.init_node('listener', anonymous=True)
    representor_pub = rospy.Publisher("PRrepresentor",PRrepresentor, queue_size=3)
    rospy.Subscriber("input_image", Image_msg, callback)

    rospy.spin()


    # list of image names
    # images = ['data//frame00001.png']
    # qimages = ['data/frame00002.png']
    
    # extract features
    # qvecs = extract_vectors(net, qimages, transform)
    
    # # match result
    # best_match = match(vecs, qvecs)
    # print(best_match)

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.image)
    global bridge,net,transform,representor_pub
    cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
    # cv_depth = bridge.imgmsg_to_cv2(data.depth, "mono16")
    IFsuccess, vecs = extract_vectors_np(net, cv_image, transform)
    if not IFsuccess:
        return
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", vecs)
    # cv2.imshow("name", cv_image)
    # cv2.imshow("depth", cv_depth)
    # cv2.waitKey(30)
    pubvecs = PRrepresentor()
    pubvecs.imageHeader = data.header
    pubvecs.representor = vecs.tolist()
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", pubvecs)
    representor_pub.publish(pubvecs)



if __name__ == '__main__':
    main(sys.argv[1:])