## Machine Learning Test 2024 Summer

This test is designed to evaluate the Machine Learning skills of students in the robotics field.
This test aims to design a controller for a legged robot.

### Data:
1. Image Specifications: The original dataset contains images of size 480x640x3. You are allowed to resize these images to lower resolutions to accommodate your model's requirements.
2. Data Generation: Utilize the data_generator.py script provided to generate the necessary data for model training and evaluation.
3. Training and Testing Split: For each num_N.txt file in the dataset, use the first 80% of the data for training your model. The remaining 20% should be used for testing its performance.
4. Dataset Utilization: While it's recommended to use the entire dataset for a comprehensive training approach, you may choose to focus on a specific pair of files from the dataset to limit the number of objects in the image, depending on your model's complexity and training capacity.

### Model Training and Evaluation: 
1. Develop your model or download the SOTA using PyTorch, ensuring it's capable of generating high-quality label images from the input data.
2. After training, use the generate.py script to generate images based on the test set. 
Make sure to save these generated images for evaluation purposes.

### Others:
1. There is no fixed deadline for this test. Our primary goal is to identify talented individuals with a passion for machine learning 
and a knack for innovative problem-solving. Take the time you need to showcase your best work.
2. This project could potentially lead to significant publications. If you are interested in PhD program,
this is a good opportunity.



