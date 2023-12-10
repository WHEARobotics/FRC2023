# ---
# title: "MNIST via Convolutions"
# execute:
#     echo: true
# format: 
#   html:
#     code-fold: true
#   pdf: 
#     code-fold: true
# jupyter: manta
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.15.2
#   kernelspec:
#     display_name: manta
#     language: python
#     name: manta
# ---

# +
import torch
from torch.autograd import Variable
from torch import optim
import torch.nn as nn
import torch.optim as optim

import torchvision
import torchvision.transforms as transforms
from torchvision import datasets
from torchvision.transforms import ToTensor

import torchsummary

import matplotlib.pyplot as plt
from IPython.display import SVG
import numpy as np
import time


# -

# ## A Deep Convolutional Neural Network for MNIST
#
# I don't have time to go over this step by step, so I'm just going to present the code quickly. 
#
# You'll see that, just as we defined a new class for our Xor network and our 3-layer MNIST network, we're doing the same here. 
#
# To make the code a little shorter, I define `conv1` and `conv2` as **blocks** that themselves each have a convolutional layer (`Conv2d`), use the `ReLU` activation function, and a pooling layer (`MaxPool2D`). Creating blocks like this that internally have layers and which, in turn, you layer with other blocks, is how deep-learning **architectures** are built. 
#
# ### Parameters to `Conv2d`
#
# When defining a convolutional layer (`Conv2d`) in Pytorch, the depth of the input is the number of `in_channels`. With a black-and-white image, we only have 1 input per pixel. If it were an RGB image with 3 values defining the pixel, the `in_channels` would be 3. 
#
# The `out_channels` is the number of kernels you want. Generally, you want to train a bunch of kernels, since each kernel has a receptive field of only `kernel_size * kernel_size` weights. (And remember that you'll also have 1 bias weight for each output.) 
#
# Although we talked about 3x3 pixel stuff above, I happen to know that it takes a really long time to train a 3x3 convnet on MNIST. Instead, we use a `kernel_size` (really "length of one side of kernel") of 5. 
#
# How much do you slide the window at each step? We set `stride` to 1, meaning that we just move the window 1 pixel over until we hit the end of the row and then go down 1. Setting it to a higher number will miss some of the small-scale patterns in your input, but will make for a smaller feature map. (I'm not sure I've _ever_ seen a production system where `stride` > 1 was used.) 
#
# What happens when your window hits the edge? Generally, you use `padding` to just put 0s on "the outside." Since our `kernel_size` is 5, that means that one when the middle pixel is on the edge, there are 2 pixels "outside." Thus, `padding = 2`. 
#
# ### Parameters to `MaxPool2d`
#
# If you understood the discussion of pooling layers, this should be pretty clear: our pooling layer has a receptive field of 2x2. It cuts the size of the feature map by 3/4, saving only the activation that has the highest absolute value. 
#
# ### Parameters to `Linear`
#
# The convolutional layers detect the features of the input, but we need to map those into one of 10 output classes (the digits from 0-9). Going through the "shapes" of the input size, the convolutional and pooling layers, the output of the second pooling layer is [32,7,7] meaning that I need 32x7x7 weights for each of the 10 outputs (plus 1 bias weight for each output value). We create a fully-connected, aka Linear, layer: 32 x 7 x 7 x 10 + 10 = 15,690 weights. 

# +
#| echo: true 
#| code-fold: false

class MnistConv(nn.Module):
    def __init__(self):
        super(MnistConv, self).__init__()
        self.conv1 = nn.Sequential(
            nn.Conv2d(
                in_channels = 1, # Black-and-white, single value
                out_channels = 16, 
                kernel_size = 5,
                stride = 1,
                padding = 2
            ),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2)
        )
        # After pooling, output is 14x14x6 
        self.conv2 = nn.Sequential(
            nn.Conv2d(
                in_channels = 16,
                out_channels = 32,
                kernel_size = 5,
                stride = 1,
                padding = 2
            ),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size = 2)
        )
        # After pooling, output is 7x7x32
        self.out = nn.Linear(7*7*32, 10)
        
    def forward(self, x):
        x = self.conv1(x)
        x = self.conv2(x)

        # Flatten the output shape of conv2 to shape of output layer. Let PyTorch do the calc.
        x = x.view(x.size(0), -1)
        x = self.out(x)
        return x

net = MnistConv()
# -

# This is not a _very_ deep neural network, lol. It's really just 6 layers:
#
# 1) Input layer (28 x 28 pixels, only 1 byte per pixel: 1x28x28)
# 2) Feature Map 1 (5x5 receptive field, 28 x 28 features, 16 kernels: 16x28x28)
# 3) Pooling layer 1 (Reduces size to 16x14x14)
# 4) Feature Map 2 (5x5 receptive field, 14 x 14 features, 32 kernels: 32x14x14)
# 5) Pooling layer 2 (Reduces size to 32x7x7)
# 6) Classification layer (Fully connects pooling layer 2 to 10 output values: 1568x10)
#
# ![](media/mnist_conv.svg)
#
# It has fewer than 30,000 weights, that's 1/10th the size of our 3-layer MNIST neural network.

# +
#| echo: true 
#| code-fold: false


torchsummary.summary(net, (1, 28, 28))
# -

# Cross Entropy Loss is correct choice for classification (choosing among options)
criterion = nn.CrossEntropyLoss()
# Adam is a refinement of SGD and is generally a good choice
optimizer = optim.Adam(net.parameters(), lr = 0.01)

transform = transforms.Compose([transforms.ToTensor()])
trainset = torchvision.datasets.MNIST(root='./data', train=True, download=True, transform=transform)
testset = torchvision.datasets.MNIST(root='./data', train=False, download=True, transform=transform)

# +
batch_size = 64

trainloader = torch.utils.data.DataLoader(trainset, batch_size=batch_size, shuffle=True)
testloader = torch.utils.data.DataLoader(testset, batch_size=batch_size, shuffle=False)
# -

# This is the _exact same_ training loop as we used on 3-layer MNIST. Not a line of code is different:

# +
#| echo: true 
#| code-fold: false

epochs = 2
losses = []
for epoch in range(epochs):  # loop over the dataset multiple times
    running_loss = 0.0
    epoch_start = time.time()
    for i, data in enumerate(trainloader, 0):
        # get the inputs; data is a list of [inputs, labels]
        inputs, labels = data

        # zero the parameter gradients
        optimizer.zero_grad()

        # forward + backward + optimize
        outputs = net(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        losses.append(loss)
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        seconds = time.time() - epoch_start
    print(f"Epoch {epoch} : loss = {running_loss/len(trainset):.6f}, took {seconds:.1f} seconds")
    running_loss = 0.0


# -

def prediction_for_image(net, q_image):
    net.eval()
    outputs = net(q_image.unsqueeze(0))
    # outputs has a batch dimension, so get the values at dim 0
    vals = outputs.squeeze().detach().numpy()
    index_of_max_val = np.argmax(vals)
    labels_for_ix = { 
        0 : "Zero",
        1 : "One",
        2 : "Two", 
        3 : "Three", 
        4 : "Four", 
        5 : "Five", 
        6 : "Six", 
        7 : "Seven",
        8 : "Eight",
        9 : "Nine"
    }
    return labels_for_ix[index_of_max_val]


# You may remember this hard-to-get target:

# +
#| echo: true
#| code-fold: false

def show_and_tell(net, dataset, index):
    query_image, ground_truth = dataset[index]
    
    # The image is now a PyTorch Tensor, so you need to convert it to a NumPy array first
    image = query_image.numpy()
    
    # The data is a 1x28x28 array, you need to remove the extra dimension
    image = image.squeeze()
    
    # Display the image and its label
    plt.imshow(image, cmap='gray')
    plt.title('Label: %i' % ground_truth)
    plt.show()

    prediction = prediction_for_image(net, query_image)
    print(f"Prediction is \"{prediction}\", ground truth is \"{ground_truth}\"")

# testset[478] is a hard one! Your model may get it wrong!
show_and_tell(net, testset, 478)
# -

# ## What's our overall accuracy?

# +
#| echo: true
#| code-fold: false

loss = np.array([])
net.eval()
for i, data in enumerate(testloader, 0):
    # get the inputs; data is a list of [inputs, labels]
    inputs, ground_truths = data
    predictions = net(inputs)
    batch_loss = torch.eq(predictions.argmax(dim=1), ground_truths.argmax(dim=0)).float()
    loss = np.append(loss, batch_loss.detach().numpy())
accuracy = 1.0 - np.mean(loss.flatten())
print(f"Across {len(testloader) * batch_size} test images, accuracy is {accuracy:.2%}")

# -

# In the case of MNIST, the convolutional neural network performs with essentially the same accuracy as the 3-layer neural network: generally achieving 92-93% accuracy. This is despite having fewer than 10% of the weights, which effectively means using much less memory. On the other hand, this deeper neural network trains a little slower, as convolving (sliding the window) is an inner loop that has to happen (even if we don't have to explicitly write the code). The depth also means that the code has a little less **cache coherence**, which is a factor in performance tuning.
#
# Pragmatically, the weight/memory savings *vastly* outweighs the slight speed hit. Convolutional neural networks are the standard architecture for neural network-based vision applications.


