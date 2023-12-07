import torch
from torch.autograd import Variable
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
from IPython.display import SVG
# %matplotlib inline
#torch.manual_seed(42)

inputs = torch.Tensor([[0,0],[0,1], [1,0], [1,1]])
targets = torch.Tensor([0,1,1,0]).view(-1,1)
inputs, targets


# +
# A simple 3-layer neural network:
# lin1 : Input layer
# lin2 : Middle layer
# Final layer (3rd) is automatically done by Pytorch from `output_dim`

class XOR(nn.Module):
    def __init__(self, input_dim = 2, output_dim=1):
        super(XOR, self).__init__()
        self.lin1 = nn.Linear(input_dim, 2)
        self.lin2 = nn.Linear(2, output_dim)
        # Set the weights to random values in range [0,1]
        self.lin1.weight.data.normal_(0, 1)
        self.lin2.weight.data.normal_(0, 1)

        print("Initially, weights are random:")
        for weight_tensor in self.state_dict():
            print(f"{weight_tensor} : {self.state_dict()[weight_tensor]}")
        
    def forward(self, x):
        x = self.lin1(x)
        x = F.sigmoid(x)
        x = self.lin2(x)
        return x

# Create an instance of the class XOR we just defined
model = XOR() 
# -

SVG('<svg xmlns="http://www.w3.org/2000/svg" version="1.1" width="450" height="500"> \
  <circle cx="100" cy="150" r="4" stroke="red" stroke-width="2" fill="none" />\
  <circle cx="250" cy="150" r="4" stroke="red" stroke-width="2" fill="none" />\
  <text x="92" y="305" font-size="24pt">σ</text>\
  <text x="240" y="305" font-size="24pt">σ</text>\
  <circle cx="175" cy="450" r="4" stroke="red" stroke-width="2" fill="none" />\
  <text x="300" y="225" stroke="red">1.0 ("Bias" weights)</text> \
  <line x1="295" y1="230" x2 = "110" y2 = "295" stroke="red" stroke-width = "2"/> \
  <line x1="298" y1="233" x2 = "255" y2 = "285" stroke="red" stroke-width = "2"/> \
  <text x="50" y="225" stroke="black"> lin1</text> \
  <line x1="100" y1="160" x2="100" y2="285" stroke="red" stroke-width="2"/>\
  <line x1="250" y1="160" x2="250" y2="285" stroke="red" stroke-width="2"/>\
  <line x1="105" y1="160" x2="245" y2="285" stroke="red" stroke-width="2"/>\
  <line x1="245" y1="160" x2="105" y2="285" stroke="red" stroke-width="2"/>\
  <text x="50" y="380" stroke="black"> lin2</text> \
  <text x="300" y="380" stroke="red">1.0 ("Bias")</text> \
  <line x1="295" y1="385" x2 = "190" y2 = "445" stroke="red" stroke-width = "2"/> \
  <line x1="100" y1="310" x2="170" y2="440" stroke="red" stroke-width="2"/>\
  <line x1="250" y1="310" x2="180" y2="440" stroke="red" stroke-width="2"/>\
</svg>')


# # Activation function
#
# We use an old favorite called "sigmoid"
#
# ## $$ \frac{\mathrm{1} }{\mathrm{1} + e^{-x}}  $$  

# +
def sigmoid_fn(sum_of_input_times_weight):
 return 1/(1 + np.exp(-sum_of_input_times_weight))
    
x = np.linspace(-10,10, 100)
plt.plot(x, sigmoid_fn(x))


# -

# ## What's the slope of the sigmoid function? Turns out to be: 
#
# ## $$ \sigma'(x)=\frac{d}{dx}\sigma(x)=\sigma(x)(1-\sigma(x)) $$

# +
def derivative_of_sigmoid(x):
    df = sigmoid_fn(x) * (1 - sigmoid_fn(x))
    return df

x = np.linspace(-10,10, 100)
plt.plot(x, derivative_of_sigmoid(x))
plt.annotate('x < 0 ? slope is +', xy=(-2.5, 0.15),
             xytext = (-10, 0.16),
            arrowprops=dict(facecolor='black'))
plt.annotate('x > 0? slope is -', xy=(2.5, 0.12), xytext=(6, 0.16), arrowprops=(dict(facecolor='black')))
plt.suptitle("Derivative of activation function")
plt.title("Gradient descent == 'Go down hill'")
plt.show()


# -

# # "Inference" or making a prediction...
#
# * You have your layers, each of which contains a bunch of weights.
# * You have your `inputs`, each of which is a set of values in the range [0,1] (in our case, our inputs are `[[0,0], [0,1], [1,0], [1,1]]`, which can be interpreted as `True` or `False`: $$ [ [F, F] , [F, T], [T, F], [F, F]] $$
# * (You also have your `targets`, the desired output for any input. But put that aside for a sec...)
#
# ## To make a prediction...
#
# 1) Multiply the value of each input by its weights in the first layer and add the weight from the first bias layer
# 2) Sum the results for each element in the second layer
# 3) Convert the results to the range [0,1] by applying the sigmoid function
# 4) Multiply each value you get from ^ by the weights in the second layer and add the weight from the second bias layer
# 5) Sum the results
#
# This is what is being done by `XOR.forward()`:
#
# ```
# def forward(self, x):
#         x = self.lin1(x)
#         x = F.sigmoid(x)
#         x = self.lin2(x)
#         return x
# ```
#
# With random weights, you end up with random output:

# +
def xor_inference(input):
    model.eval()
    t = torch.Tensor(input)
    net_input = Variable(t, requires_grad=False)
    output = model(net_input)
    return output

print("Initially, with random weights:")
for input in [[0,0], [0,1], [1,0], [1,1]]:
    output = xor_inference(input)
    print(f"{input} -> {output[0]:.2f}")
# -

# # Training
#
# ## How to define a mistake? 
#
# "Mean Squared Error" is the average of the mistakes, squared. We can use other loss functions, but MSE is a good choice for this problem.

loss_func = nn.MSELoss()

# ## How to figure things out? Stochastic Gradient Descent
#
# We've talked about gradient descent. "Stochastic" gradient descent just means that you do it a little at a time. There are other variations on gradient descent, but "SGD" is common.
#
# It needs to know about the `model`, but it also needs to know how quickly to adjust the weights every step. This is the **learning rate** (`lr` below). To avoid getting stuck in a local minimum, you don't just throw away your previous adjustment, you use maybe 90% to "keep you going" (`momentum` below).

optimizer = optim.SGD(model.parameters(), lr=0.02, momentum=0.9)

# ## How to change weights / descend hill / minimize loss? 
#
# * Take a look at the data, let's say, 501 times. Each time we look at all the data, it's called an `epoch`.
# * Call taking a single input value and dealing with it a `step`
# * grab any one of the inputs at random
# * Call the input values of that particular input `x_var`
# * Call the target value of that particular input `y_var`
# * Forget what you know about the slope from last time (zero out the gradients)
# * Make an inference. (See above: push the inputs through the big bag of weights...) Call the result `y_hat`
# * Your gross mistake is the difference between your prediction (`y_hat`) and your target (`y_var`)
# * Pytorch _automatically_ calculates the slope/gradient of your mistake by using the derivative of the function you are using to do the inference ("automatic differentiation") and adjusts the weights accordingly. "Accordingly": based on the slope times the weight times the learning rate, plus the momentum you have from previous adjustments. Below, this is what's happening in `loss.backward()`
# * Do it again. Hopefully, your total "loss" (the difference between your predictions and your hoped-for targets) goes down.

epochs = 2001
steps = inputs.size(0)
losses = []
for i in range(epochs):
    for j in range(steps):
        data_point = np.random.randint(inputs.size(0))
        x_var = Variable(inputs[data_point], requires_grad=False)
        y_var = Variable(targets[data_point], requires_grad=False)
        
        optimizer.zero_grad()
        y_hat = model(x_var)
        loss = loss_func.forward(y_hat, y_var)
        losses.append(loss.detach().numpy())
        loss.backward()
        optimizer.step()
        
    if i % 100 == 0:
        print(f"Epoch: {i}, Loss: {loss.data.numpy()}")

# # Visualizing the gradient descent
#
# Our "altitude" going down the hill is our `loss` at every step. We kept track of those, so we can look at how we did as we trained. We have to "wander around" quite a bit before focusing in on the right path. Let's plot both the raw loss and a line showing the average loss over a window of 100 steps. 

plt.plot(np.arange(len(losses)), losses)
# Calculate the moving average
cumsum_vec = np.cumsum(np.insert(losses, 0, 0)) 
moving_averages = (cumsum_vec[100:] - cumsum_vec[:-100]) / 100
plt.gca().set_ylim([0,1])
plt.plot(np.arange(len(losses)-99), moving_averages)


# # Training has changed the weights so they (probably) solve the problem
#
# Compare the weights in the model to where they were when the model was first created ("Initially, weights are random:") 

print("After training, weights are not random (although they may still be hard to interpret!)")
for weight_tensor in model.state_dict():
    print(f"{weight_tensor} : {model.state_dict()[weight_tensor]}")

torch.Tensor([-5.1, 3.8])*torch.Tensor([-2, 1.8])*torch.Tensor([-2.8,0.25])

print("And the new weights (probably) give the desired result")
for input in [[0,0], [0,1], [1,0], [1,1]]:
    output = xor_inference(input)
    
    print(f"{input} -> {output[0]:.2f}")

# ## There are lots of ways to set the weights so that they solve this problem! Run this notebook and compare your results with your colleagues'. 
#
# Although the weights are different, is there a pattern (or patterns) that you see?

# ## Restart the kernel and rerun to get different weights: 
#
# Once you've run the notebook, you can run it again by going to the "Kernel" menu in this browser window and choosing "Restart" and then running the cells one-by-one. Or you can choose "Restart and run all cells..." to execute the entire notebook. 
#
# If you lower the number of epochs to, say, 500, different runs will sometimes not fully solve the problem but might be "pretty good" or work for 3 of the 4 inputs or something like that.

# # Review
#
# **Boolean logic** is a type of math where results are either **True** or **False**. Instead of addition, multiplication, etc., the operators in Boolean logic are operations like **AND** (which is **True** if-and-only-if the two inputs are **TRUE**) and **OR** (**TRUE** if either or both the inputs are **TRUE**). Another operator is **XOR** (aka Exclusive-Or, which is **TRUE** if either BUT NOT BOTH inputs are **True**). In Computer Science, the values 0 and 1 are often substituted for **False** and **True**.
#
# \begin{array}{|c|c|c|}
# \hline
# A & B & A \text{ XOR } B \\
# \hline
# 0 & 0 & 0 \\
# 0 & 1 & 1 \\
# 1 & 0 & 1 \\
# 1 & 1 & 0 \\
# \hline
# \end{array}
#
# You wrote a 3-layer artificial neural network that correctly calculates **XOR**.

# The **layers** of the artificial neural network consist of a **Tensor** of **weights**. A tensor is a big rectangle of numbers, but instead of the 2 dimensions of a rectangle, or the 3 dimensions of a box (which kind-of is a "3-dimensional rectangle"), a tensor can be many dimensions. 
#
# (Pedantically, a single value is a **scalar**, which is a tensor of **rank** (or dimension) 0. An array of values is a **vector**, or a tensor of rank 1.  A grid of values is a **matrix**, or a tensor of rank 2.)

# The number of weights in a layer is equal to the number of inputs to that layer multiplied by the number of outputs in that layer plus a bias weight for each element in the outut. To rephrase it, for every value in the input you have as many weights as the length of your output and a single bias weight. A third way of viewing it is that each _output_ has as many "inbound weights" as the number of input values plus a bias weight.

# To make a prediction (or **inference**) with an artificial neural network, you start with an input item (in the case of XOR, a rank 1 tensor, for instance `[1, 0]`). For each output in the layer, you sum each input value times each inbound weight value. You then apply an **activation function** such as the **sigmoid** function to the total sum. You repeat this process for every layer in the neural network. The end result is, in the case of XOR, a rank 0 tensor (that is, a scalar value) that is in the range `[0,1]`, which can be thought of as "whether the network 'thinks' the inputs are **True** or **False**." Outputs near `0.0` and `1.0` are "pretty confident predictions" in the result, while an output near 0.5 would mean "the network thinks it's a coin flip whether this is **True** or **False**".

# With the random weights you start with, the predictions of the artificial neural network are wrong, in a random manner.

# To **train** the network, you use **Stochastic Gradient Descent** to calculate the **loss** (error) that occurs when you try to make a prediction. The activation function and its **derivative** are chosen because they have characteristics that allow for automated gradient descent. Once you know the loss, you adjust the weights slightly, based on the gradient, and try again. You do this over and over. If the network **converges**, the loss goes to 0 and you have a potentially useful solution.

# If you train the XOR neural network for enough epochs, it will eventually converge and you will have a **model** on which you can do successful inference. 
#
# # You've created a system that learned how to solve a problem


