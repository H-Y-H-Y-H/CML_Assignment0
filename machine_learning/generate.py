import torch
from model import *
import matplotlib.pyplot as plt


def generate_image(input_image_path, save_image_path, model_path='yourmodels.pth', device='cuda'):
    # Load your model
    model =.to(device)
    model.load_state_dict(torch.load(model_path))
    model.eval()

    # Load an image

    output_image = model(input_image)

    # Save the output image to the results folder.

    plt.imsave(save_image_path + ' .png', output_image)


if __name__ == '__main__':
    # please generate at least 10 images.

    input_image_path = 'path/to/your/input/ .png'
    save_image_path = 'results/'
    generate_image(input_image_path, save_image_path)