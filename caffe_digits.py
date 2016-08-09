#!/usr/bin/env python
import numpy as np

import sys
caffe_root = '../root/caffe/'  
sys.path.insert(0, caffe_root + 'python')

import caffe

caffe.set_mode_cpu()

model_def = 'caffe_model/deploy.prototxt'
model_weights = 'caffe_model/snapshot_iter_21120.caffemodel'

net = caffe.Net(model_def,      # defines the structure of the model
                model_weights,  # contains the trained weights
                caffe.TEST)     # use test mode (e.g., don't perform dropout)

# set the size of the input (we can skip this if we're happy
#  with the default; we can also change it later, e.g., for different batch sizes)
net.blobs['data'].reshape(1,        # batch size
                          1,         # 3-channel (BGR) images
                          28, 28)  # image size is 227x227

image = caffe.io.load_image('digit_images/mnist_complete_zero.png')



# copy the image data into the memory allocated for the net
net.blobs['data'].data[...] = image

### perform classification
output = net.forward()

output_prob = output['prob'][0]  # the output probability vector for the first image in the batch

print 'predicted class is:', output_prob.argmax()
