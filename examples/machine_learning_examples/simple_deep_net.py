"""
Refs
1. Jonathan Huis blog: https://jhui.github.io/2018/02/09/PyTorch-neural-networks/
2. Basic Operations: https://jhui.github.io/2018/02/09/PyTorch-Basic-operations/
"""
import torch
from torch.autograd import Variable
import numpy as np

def test_gradient(): 
    """
    1. Variable wraps a tensor, so supports nearly all APIs for the tensor, as well as backward propagation
        - by default, requires_grad = False. requires_grad means "trainable"
        - once you turn this off, requires_grad = off, the subgraphs will be "untrainable" too
    2. Tensor: like np array. 
        - all functions ends with _ (like zero_()) is inplace operation
        - tensor[:, 0] = 0 is valid
        - conversion: t = torch.from_numpy(arr); arr = t.numpy()
        - view is like array.reshape
    - "tensor" = "np array"
    - "backbone" = feature extractor (feature map)
    - "autograd": pytorch uses this to calculate gradient
    3. tensor can calculate gradient, using Autograd
        - do cost.backward()    # calculate gradient using the dynamic computation graph 
        - then x.grad()     # returns 2x2 gradient
    4. dynamic computation graph will store: function that creates variable (x doesn't have one, but y and z do)
    5. Terms 
        - "tensor" = "np array"
        - "backbone" = feature extractor (feature map)
        - "autograd": pytorch uses this to calculate gradient

    """
    # Variables wrap a Tensor
    x = Variable(torch.ones(2, 2), requires_grad=True)
    print("Variable: ", x)
    x.requires_grad = False
    print("Variable turns off requires_grad: ", x)
    x.requires_grad = True

    # 2
    # adding 2 to all elements
    y = x+2
    print("y adding 2: ", y)
    z = y*y*2
    print("z: ", z)
    out = z.mean()
    print("z's mean: ", z.mean())
    print("z's norm: ", z.norm())
    print("z reshaped (view): ", z.view(1,4))

    # 3 
    # do the math your self: del(out)/del(x_i) = 3
    out.backward()
    print("gradient wrt x: ", x.grad)
    
    # 4
    print("dynamic computation graph x: ", x.grad_fn)
    print("dynamic computation graph y: ", y.grad_fn)

    # 5 numpy
    arr = np.array([1,2,3])
    t = torch.from_numpy(arr)
    print("t from numpy: ", t)
    print("arr from tensor: ", t.numpy())

def test_backward_propagation(): 
    """
    1. Procedure: 
        1. calculate output
        2. then loss 
        3. get gradient of error wrt x 
        4. reset gradient for the next iteration
    2. You can see error will stablize at one place. So you want to have adaptive learning rate 
    3. Basic data types: 
        1. w1.data to access data for multiplication, etc.
    4. loss.backward(gradient)
        - loss is always assumed to be a scaler. If you call backward on a tensor( say 1x3 ), Pytorch assumes that this tensor is an intermediate
        - So PyTorch is assuming del(L)/del(x) = del(L)/del(intermediate) * del(intermediate)/del(x). 
        - So you should provide del(L)/del(intermediate)
    5. ReLu: y = 0  if x < 0. So we need this for forward and backward passes
    """
    class MyReLu(torch.autograd.Function): 
        @staticmethod
        def forward(ctx, input): 
            ctx.save_for_backward(input)
            return input.clamp(min = 0)
        @staticmethod
        def backward(ctx, grad_output): 
            input, = ctx.saved_tensors
            grad_input = grad_output.clone()
            # np[arr < 0] just give you list of elements < 0
            grad_input[input < 0] = 0
            return grad_input

    dtype = torch.FloatTensor
    # input: 2 x 4; out: 2 x 3; output = input x w1 x w2
    input = Variable(torch.randn(2, 4).type(dtype), requires_grad = False)
    output = Variable(torch.randn(2, 3).type(dtype), requires_grad = False)

    w1 = Variable(torch.randn(4, 5).type(dtype), requires_grad = True)
    w2 = Variable(torch.randn(5, 3).type(dtype), requires_grad = True)

    learning_rate = 1e-10
    for i in range(500): 
        # Clamps all elements in input into the range [ min, max ]
        # y_pred = input.mm(w1).clamp(min=0).mm(w2)
        my_relu = MyReLu.apply
        # aliasing the function as my_relu
        # you can see the output is simply 0 now.
        y_pred = my_relu(input.mm(w1).clamp(min=0)).mm(w2)
        error = (y_pred - output).pow(2).sum()
        # print("error: ", error)
        error.backward()
        w1.data -= - learning_rate * w1.grad.data
        w2.data -= - learning_rate * w2.grad.data

        w1._grad.data.zero_()
        w2._grad.data.zero_()
    print("w1 data: ", w1.data)

def test_small_nn(): 
    """
    1. Terms 
        - Affine Layer: fully connected layer
        - Conv Layer: apply kernel across image 
            - problem: not rotation, scale invariant. 
        - Pooling layer: to downsample right after conv layer for small-scale rotation & scale invariance
            1. a typical network may look like: 
                1. input image 
                2. conv layer 
                3. non-linearity (ReLU)
                    - Without non-linear layer, the network is purely linear, and canNOT address non-linearity
                4. pooling 
            2. pooling size is always 2x2, so 2 on each dimension
                1. average pooling: average of each patch 
                2. Max pooling: max of each patch
    2. in init, define layers, which are basically computational graphs
        - Functionals include ReLu and maxpoolings. 
    3. Need forward(x)
        - x is a minibatch: 
            [
            [img1_R, img1_G, img1_B], 
            ...
            ]
    """
    import torch.nn as nn
    import torch.nn.functional as F 

    class MyNet(nn.Module): 
        def __init__(self): 
            # 1 input image channel, 6 output channels; 5x5 conv kernel
            super(MyNet, self).__init__()
            # in, out, kernel size
            self.conv1 = nn.Conv2d(1,6,5)
            # 16 output channels. Each channel output is (input_size + 2*padding - kernel) +1
            self.conv2 = nn.Conv2d(6,16,5)
            #
            # # fc is fully connected. say 5 for the output
            self.fc1 = nn.Linear(16 * 5 * 5, 120)
            self.fc2 = nn.Linear(120, 64)
            self.fc3 = nn.Linear(64, 10)

        def forward(self, x): 
            # Max pooling, 2x2
            # see [1, 1, 32, 32]
            print(x.size())
            x = F.max_pool2d(F.relu(self.conv1(x)), (2,2))
            # 2 is same as 2x2
            # see [1, 6, 14, 14], where 14 = (32 - 5 + 1)/2
            print(x.size())
            x = F.max_pool2d(F.relu(self.conv2(x)), 2)
            # see [1, 16, 5, 5 ], where (14 - 5 + 1) /2 = 5
            print(x.size())

            # see [1, 400] 
            x = torch.flatten(x, start_dim=1)
            print(x.size())
            x = F.relu(self.fc1(x))
            x = F.relu(self.fc2(x))
            x = self.fc3(x)
            # see [1,10]
            print("final x size: ", x.size())
            return x
    
    n = MyNet()
    params = list(n.parameters())
    print("params: ", len(params))  # 10 sets of trainable params, in which first [6,1,5,5], [6], [16,6,5,5], [16]... [6], [16] are weights for combinding the convolved channels together
    print("params: ", params[3].size())

    input = Variable(torch.randn(1,1,32,32))
    out = n(input)

    target = Variable(torch.arange(1, 11))   # Create a dummy true label Size 10.
    print("targets: ", target)
    criterion = nn.MSELoss()
    loss = criterion(out, target)
    # You can technically trace the graph by calling next_functions.
    print(loss.grad_fn.next_functions[0][0])
    import torch.optim as optim
    optimizer = optim.SGD(n.parameters(), lr=0.001, momentum=0.9)
if __name__ == "__main__": 
    # test_gradient()
    # test_backward_propagation()
    test_small_nn()
