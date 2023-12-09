import torch
import torchvision
import torchvision.transforms as transforms
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
import numpy as np
import time 

# # MNIST: Recognizing hand-written digits (from the US Post Office!)
#
# This notebook builds off the concepts introduced in **xor.py**. 
#
# This is a very famous challenge. Solving this challenge by writing traditional `if-then`-style code is very difficult because there is so much variation between how folks might write a digit. It's so famous that getting the dataset is built into PyTorch!
#
# The following cell will download and extract the dataset. 

# Load MNIST dataset
transform = transforms.Compose([transforms.ToTensor()])
trainset = torchvision.datasets.MNIST(root='./data', train=True, download=True, transform=transform)
testset = torchvision.datasets.MNIST(root='./data', train=False, download=True, transform=transform)

# The **training set** is what we will use to adjust the weights in our network. The **testset** is held back from training and is used to see how well the network is doing. We couldn't separate train and test in XOR because we just had a grand total of four inputs (`[[0,0], [0,1], [1,0], [1,1]]`).
#
# Each element in each of these sets consists of a 28x28 grid and a label which is the **label** or **ground truth** of how that grid should be interpreted. 
#
# The `label`'s easy to understand:

data, label = trainset[0]
label

# But the `data` is a big rectangle of numbers in the range [0,1]. It's a **tensor**. Here's 1 of the 28 rows in an input:

data[0][10]

# That's hard to understand, but we can visualize the data. You can see that some of these are pretty easy to interpret and some might be a little harder:

# +
fig, axes = plt.subplots(4, 4, figsize=(6,6))

for i, ax in enumerate(axes.flat):
    # Get the i-th sample from the training set
    image, label = trainset[i]

    # The image is now a PyTorch Tensor, so you need to convert it to a NumPy array first
    image = image.numpy()

    # The data is a 1x28x28 array, you need to remove the extra dimension
    image = image.squeeze()

    # Display the image and its label
    ax.imshow(image, cmap='gray')
    ax.set_title('Label: %i' % label)
    ax.axis('off')

plt.tight_layout()
plt.show()

# -

# # Batches
#
# While a 28x28 rectangle of values is more than we can deal with in a single effort, it's not very large by computer memory standards. Modern computers can fit a bunch of these rectangles and their labels in memory and work on them in parallel. Instead of working on a single image at a time, we can work on a **batch** of them. 
#
# The gradients of the entire batch are used to figure out the weight adjustments. *Generally*, bigger batches train faster because the system adjust the weights to help the average performance of the batch. But if `batch_size` is too large, you'll run out of memory. And sometimes large batches *slow down* training, because the batch has so much variation it's "average slope" is misleading and isn't actually heading towards the overall minimum.

# In PyTorch, you use the `DataLoader` class to quickly move data in and out of CPU or GPU memory. You shuffle the training data so that each time you grab a batch you train on a a different subset of inputs. That's not necessary for the test data since the test data is never used for training.

# +
batch_size = 64

trainloader = torch.utils.data.DataLoader(trainset, batch_size=batch_size, shuffle=True)
testloader = torch.utils.data.DataLoader(testset, batch_size=batch_size, shuffle=False)
# -

# # Activation function: instead of sigmoid, one called RELU
#
# This is a simple "piecewise function": 
#
# $$ relu(x) = 
# \begin{cases} 
# 0 & \text{if } x < 0 \\
# x & \text{if } x \geq 0 
# \end{cases}  $$
#
# This **Rectified Linear Unit** (RELU) (can you believe the fancy name?) is *ultra*-fast to compute and is the most common activation function used in deep neural networks.

x = np.linspace(-2, 2, 100)
plt.plot(x, np.maximum(0,x))

# # What's the slope of RELU? Turns out to be:
#
# $$ relu'(x) = 
# \begin{cases} 
# 0 & \text{if } x < 0 \\
# 1 & \text{if } x \geq 0 
# \end{cases}  $$
#
# This is also blisteringly fast to compute even with huge amounts of weight. 
#
# Remember: both the activation function and it's derivative (that is, the function that describes it's slope) are important to gradient descent, so we should always have a sense of both.

plt.plot(x, np.where(x < 0, 0, 1))


# # A simple neural net to solve MNIST
#
# Notice how similar this is to our XOR neural network! It's just a bigger box of weights!
#
# Again we have 3 layers:
#
# 1) A 28*28 input layer
# 2) A 500-element middle layer
# 3) A 10-element output layer
#
# The values in the input layer are the values that correspond to the grayscale handwwritten-digit.
#
# The values in the output layer are the index of the desired label (index 0 indicates label "0", index 1 indicates label "1", etc.)
#
# The `forward` function:
#
# 1) Flattens the 28x28 rectangle of values into a single array of length 784 (28x28) with the `view` function
# 2) Multiples every value in the input with the weights  of the `lin1` layer + the first layer's bias weights (this is the exact same code we used in XOR! Just more weights!) 
# 3) Applies the activation function with `torch.relu()` (in XOR we used `sigmoid()`)
# 4) Multiplies the output of ^ with the weights of the `lin2` layer + the second layer's bias weights (again, exactly the same as we did in XOR)

# +
class Mnist(nn.Module):
    def __init__(self):
        super(Mnist, self).__init__()
        self.lin1 = nn.Linear(28*28, 500)
        self.lin2 = nn.Linear(500, 10)

    def forward(self, x):
        x = x.view(-1, 28*28)
        x = self.lin1(x)
        x = torch.relu(x)
        x = self.lin2(x)
        return x

net = Mnist()
# -

# XOR had 6 weights (4 in `lin1`, 2 in `lin2`]. This model has 397,000!:

# +
input_size = 28 * 28
middle_layer_size = 500
output_size = 10

interconnect_weights_in_lin1 = input_size * middle_layer_size
bias_weights_in_lin1 = middle_layer_size
weights_in_lin1 = interconnect_weights_in_lin1 + bias_weights_in_lin1

interconnect_weights_in_lin2 = middle_layer_size * output_size
bias_weights_in_lin2 = output_size
weights_in_lin2 = interconnect_weights_in_lin2 + bias_weights_in_lin2

total_weights = weights_in_lin1 + weights_in_lin2
weights_in_lin1, weights_in_lin2, total_weights
# -

# There's a handy library that will do the above calculations for us and make things pretty:

# +
from torchsummary import summary

summary(net, (1, 28, 28))
# -

# We used "Mean Squared Loss" for XOR, where we had one output. 
# "Cross-Entropy Loss" is used in "choice between alternatives" problems. In this case, we want to choose between the 10 digits.
criterion = nn.CrossEntropyLoss()

# Just like XOR, but a slower learning rate (harder problems require slower learning rates)
optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)

# # Training the network will take time and make your computer sound like a helicopter
#
# This network is adjusting over 1/3 of a million weights every batch and has over 900 batches worth of data before it's seen all the data in the training set. And that's just a single epoch!
#
# If training doesn't make the fans on your computer start running at full bore, something's probably wrong.
#
# How long it takes to train will depend on whether or not you have a Graphical Processing Unit (GPU) on your machine and the memory and speed of your machine. It may take a minute to train an epoch on your machine, it may take 10 minutes. 
#
# If you receive an out-of-memory (OOM) error or a CUDA exception, you'll need to restart the kernel, lower the `batch_size` when you create the `DataLoader`s, and try again. (Try cutting the `batch_size` in half until it runs. You can then try slowly increasing it until you're flirting with OOM errors.)
#
# Compare your compute speed with your colleagues'! On faster machines, try to train for more epochs to see if it results in lower losses. 

# +
# %%time 

# Train the network
epochs = 30
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
    print(f"Epoch {epoch} : loss = {running_loss/len(trainset):.3f}, took {seconds:.1f} seconds")
    running_loss = 0.0

print('Finished Training')
# -

weights_per_batch = batch_size * total_weights
weight_updates_per_epoch = weights_per_batch * len(trainloader)
total_weight_updates = weight_updates_per_epoch * epochs
print(f"{total_weights:,} * {batch_size} * {len(trainloader)}  * {epochs} = {total_weight_updates:,} weight modifications")

# # Results
#
# Remember that the `testset` has handwritten digits that the network hasn't seen before, so it seems like a good test of whether the model has learned how to handle hand-written digits or not. Grab a random image:

# +
# Get an image from the test set
query_image, label = testset[11]

# The image is now a PyTorch Tensor, so you need to convert it to a NumPy array first
image = query_image.numpy()

# The data is a 1x28x28 array, you need to remove the extra dimension
image = image.squeeze()

# Display the image and its label
plt.imshow(image, cmap='gray')
plt.title('Label: %i' % label)
plt.show()
# -

# * Put the model into `evaluation` mode (doesn't bother figuring out gradients).
# * Get the results for the query image.
#
# The output layer has 10 elements in it. The value of each element in the output corresponds to how strongly the model "thinks" the input may correspond to that output. The element with the largest value is the model's "best guess." 
#
# It's interesting to find query images that are ambiguous and look at the model outputs. 

net.eval()
net(query_image)


# The fact that the index in the output is the same as the digit is just a coincidence of this dataset. You can map the index into any label that makes sense. In a manta ray identification model, index[0] might be the manta named "Queenie" and index[1] might be "Big Bertha" (or whatever). So you usually write a function that converts the raw outputs into a human-readable prediction:

# +
def prediction_for_image(net, q_image):
    net.eval()
    outputs = net(q_image)
    # outputs has a batch dimension, so get the values at dim 0
    vals = outputs.squeeze().detach().numpy()
    index_of_max_val = np.argmax(vals)
    labels_for_ix = { 
        0 : "oreZ",
        1 : "enO",
        2 : "owT", 
        3 : "eerhT", 
        4 : "ruoF", 
        5 : "eviF", 
        6 : "xiS", 
        7 : "neveS",
        8 : "thgiE",
        9 : "eniN"
    }
    return labels_for_ix[index_of_max_val]

prediction_for_image(net, query_image)


# -

# We can put grabbing the image, visualizing it, and making a prediction all together. You can quickly change the value of `index` and see how your model does with various inputs: 

# +
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

# # Accuracy
#
# Spot checks are all well and good, but let's calculate the overall accuracy against the test images (which were never seen during training):

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

# How did you do? To try to increase accuracy, there are many things you could try:
#
# * Increase the number of training epochs ("Cook it longer")
# * Lower the training rate and increase epochs ("Cook it longer and slower")
# * *Lower* the size of the middle layer from 500. This is an attempt to make each node in the middle layer "smarter" and "more of a generalist," but may require more epochs or may not converge at all

# # Review
#
# MNIST is a famous challenge in the field of machine learning. It consists of a large number of hand-written digits (60,000 training examples, 10,000 test examples). Correctly recognizing these digits was a real-world challenge for the USPS and is difficult to do with hand-written code. 
#
# You learned that the **training set** is used when modifying the weights of the model, while the **test set** is held back and only used for checking the model's quality. (In addition, in a serious application you'll have a third **validation set** that as the developer you may never see! It's used to make sure that you haven't made mistakenly (or nefariously!) "leaked" data between the test and training sets.)
#
# The artificial neural network solution to this is *extremely* similar to the solution to the XOR problem: write a 3-layer neural network, choose a loss and activation function, and train it for awhile. 
#
# However, while the code remains very simple, close to 400,000 weights need to be trained to solve this problem. To help with such large numbers, you swapped out the sigmoid activation function for the the fast-to-calculate RELU function. You also started using PyTorch's `DataLoader` class, which moves a **batch** of data in and out of memory very efficiently. Finally, you significantly benefited from the presence of a GPU on your machine (if you are fortunate enough to have one). 
#
# The resulting network does quite well. You likely achieved somewhere over 90% accuracy with only a few epochs of training. 
#
# ## You've achieved, in a matter of minutes, an accuracy that would take days or weeks to achieve by hand-coding


