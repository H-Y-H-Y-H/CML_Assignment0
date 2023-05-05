## Machine Learning Test

This test is designed to evaluate the Machine Learning skills of students in machine learning field.
The goal of this test is to design a neural network that can fit a dataset with varying input and output sizes. 
The dataset contains information about the state of objects on a table, including their position, orientation, and size before and after being moved.

### Data representation:
For each row in the .txt data file, it describes the state of objects.
(x0, y0, width0, height0, angle0), (x0, y0, width0, height0, angle0), ... \
The length of each row = num_obj *5 \
Please feel free to run data_visualization.py to see the difference between inputs and labels. \
Noted, the input size is varying. You can set a maximum input size: 5x15 --> 5 parameters x 15 objects.

### Test Instructions
Design a neural network with **Pytorch** that can fit the dataset. 
You are encouraged to leverage computer vision techniques to map the objects' state to higher dimensions like the pictures to achieve better performance.
Please feel free to use pre-trained models if needed.

### Model Performance: 
The ability of your neural network to accurately fit the dataset and generalize to new data.
Please use 70% data for training, 20% data for validation and 10% data for test.

### Report Quality:
The clarity and completeness of your report, including detailed explanations of your neural network design, training process, and testing results.

Hint: Do not treat this test as a simple supervised learning task. A good machine learning algorithm designer should also have a good intuition of the data and an ability to extract critical information from the raw dataset. 





