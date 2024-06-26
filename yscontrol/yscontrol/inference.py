import os
import time
import argparse
import sys
import datetime

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.utils.data as data
from torch.cuda import amp
from torch.utils.tensorboard import SummaryWriter
import torchvision
import numpy as np
import matplotlib.pyplot as plt
from spikingjelly.activation_based import neuron, encoding, functional, surrogate, layer


class SNN(nn.Module):
    def __init__(self, tau):
        super().__init__()

        self.layer = nn.Sequential(
            layer.Flatten(),
            layer.Linear(5, 128, bias=False),
            neuron.LIFNode(tau=tau, surrogate_function=surrogate.ATan()),
            layer.Linear(128, 64, bias=False),
            neuron.LIFNode(tau=tau, surrogate_function=surrogate.ATan()),
            layer.Linear(64, 2, bias=False),
            neuron.LIFNode(tau=tau, surrogate_function=surrogate.ATan()),
            )

    def forward(self, x: torch.Tensor):
        return self.layer(x)

def main():
    '''
    :return: None

    * :ref:`API in English <lif_fc_mnist.main-en>`

    .. _lif_fc_mnist.main-cn:

    使用全连接-LIF的网络结构，进行MNIST识别。\n
    这个函数会初始化网络进行训练，并显示训练过程中在测试集的正确率。

    * :ref:`中文API <lif_fc_mnist.main-cn>`

    .. _lif_fc_mnist.main-en:

    The network with FC-LIF structure for classifying MNIST.\n
    This function initials the network, starts trainingand shows accuracy on test dataset.
    '''
    parser = argparse.ArgumentParser(description='LIF MNIST Training')
    parser.add_argument('-T', default=100, type=int, help='simulating time-steps')
    parser.add_argument('-device', default='cuda:0', help='device')
    parser.add_argument('-data-dir', type=str, help='root dir of MNIST dataset')
    parser.add_argument('-resume', type=str, help='resume from the checkpoint path')
    parser.add_argument('-tau', default=2.0, type=float, help='parameter tau of LIF neuron')

    args = parser.parse_args()

    net = SNN(tau=2)

    print(net)

    net.to(args.device)


    train_dataset, test_dataset = DVSDataset('data.csv', 'label.csv').split(0.9)


    if args.resume:
        checkpoint = torch.load(args.resume, map_location='cpu')
        net.load_state_dict(checkpoint['net'])

    
    

    encoder = encoding.PoissonEncoder()
    
    net.eval()
    # 注册钩子
    output_layer = net.layer[-1] # 输出层
    output_layer.v_seq = []
    output_layer.s_seq = []

    test_acc = 0
    T=0
    k=0
    with torch.no_grad():
        img,label = test_dataset[:100]
        # for img,label in test_dataset:
        img = img.to(args.device)
        label = label.to(args.device)
        k+=1
        out_fr = 0.
        for t in range(args.T):
            encoded_img = encoder(img)
            out_fr += net(encoded_img)
        out_fr = out_fr / args.T
        test_acc += (out_fr.argmax(1) == label).float().sum().item()
    # print(T/k)
    print(test_acc/len(label))
    
def inf(data):
    net = SNN(tau=2.0)
    net.to('cuda:0')
    
    checkpoint = torch.load('./checkpoint_max.pth', map_location='cpu')
    net.load_state_dict(checkpoint['net'])
    
    encoder = encoding.PoissonEncoder()
    
    net.eval()
    output_layer = net.layer[-1] # 输出层
    output_layer.v_seq = []
    output_layer.s_seq = []
    
    out_fr = 0.
    for t in range(100):
        encoded_img = encoder(data)
        out_fr += net(encoded_img)
    out_fr = out_fr / 100
    print(out_fr.argmax(1).item())
    return out_fr.argmax(1).item()

if __name__ == '__main__':
    inf(torch.Tensor([-2.6,0.,0.,-2.,0.09]).unsqueeze(0).to('cuda:0'))
