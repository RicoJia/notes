"""
Refs
1. Jonathan Huis blog: https://jhui.github.io/2018/02/09/PyTorch-neural-networks/
"""
import torch
from torch.autograd import Variable

def test_gradient(): 
    """
    1. Variable wraps a tensor, so supports nearly all APIs for the tensor, as well as backward propagation
        - by default, requires_grad = False. requires_grad means "trainable"
        - once you turn this off, requires_grad = off, the subgraphs will be "untrainable" too
    2. Tensor: like np array. 
    3. tensor can calculate gradient, using Autograd
        - do cost.backward()    # calculate gradient using the dynamic computation graph 
        - then x.grad()     # returns 2x2 gradient
    4. dynamic computation graph will store: function that creates variable (x doesn't have one, but y and z do)
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

    # 3 
    # do the math your self: del(out)/del(x_i) = 3
    out.backward()
    print("gradient wrt x: ", x.grad)
    
    # 4
    print("dynamic computation graph x: ", x.grad_fn)
    print("dynamic computation graph y: ", y.grad_fn)

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
    """

    dtype = torch.FloatTensor
    # input: 2 x 4; out: 2 x 3; output = input x w1 x w2
    input = Variable(torch.randn(2, 4).type(dtype), requires_grad = False)
    output = Variable(torch.randn(2, 3).type(dtype), requires_grad = False)

    w1 = Variable(torch.randn(4, 5).type(dtype), requires_grad = True)
    w2 = Variable(torch.randn(5, 3).type(dtype), requires_grad = True)

    learning_rate = 1e-10
    for i in range(500): 
        # Clamps all elements in input into the range [ min, max ]
        y_pred = input.mm(w1).clamp(min=0).mm(w2)
        error = (y_pred - output).pow(2).sum()
        # print("error: ", error)
        error.backward()
        w1.data -= - learning_rate * w1.grad.data
        w2.data -= - learning_rate * w2.grad.data

        w1._grad.data.zero_()
        w2._grad.data.zero_()
    print("w1 data: ", w1.data)

if __name__ == "__main__": 
    # test_gradient()
    test_backward_propagation()
