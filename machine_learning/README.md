## Machine Learning Test

This test is designed to evaluate the Machine Learning skills of students in the machine learning field.
This test aims to design a neural network that can fit a dataset with varying input and output sizes. 
The dataset contains information about the state of objects on a table, including their position, orientation, and size before and after being moved.

### Data representation:
For each row in the .txt data file, it describes the state of objects.
(x0, y0, width0, length0, angle0), (x1, y1, width1, length1, angle1), ... \
The length of each row = num_obj *5 \
Please run data_visualization.py to see the difference between inputs and labels. \
Noted, the input size is varying. You can set a maximum input size: 5x15 --> 5 parameters x 15 objects.

### Model Performance: 
The ability of your neural network to accurately fit the dataset and generalize to new data.
Please use 70% data for training, 20% data for validation and 10% data for test.

### Report Quality:
The clarity and completeness of your report, including detailed explanations of your neural network design, training process, and testing results.

### Test Instructions
1. Design a neural network with **Pytorch** that can fit the dataset. 
2. If you visualize the dataset or use correlation analysis, you can easily find out that the x y values of the input are not necessary. You can eliminate these two numbers when you feed the input into the model.
3. Use Transformer architecture, Auto regression, and Gaussian Mixture Model.
4. The input of your model is a list of object states: [(width0, length0), (width1, length1), ... (widthN,lengthN)].
5. The output of your model is a list of object positions. [(x0, y0), (x1, y1), ... (xN,yN)].
6. Use GMM, so that your model output is not deterministic.



