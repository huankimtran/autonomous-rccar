import pathlib
import numpy as np
from tensorflow.keras.preprocessing.image import load_img, img_to_array
import matplotlib as mpl
import matplotlib.pyplot as plt
import cv2

def load_all_image_from_folder(dir_path, ext=['*.jpg','*.png','*.bmp'], rescale=1.0/255.0, to_gray=False):
    """
        ext is extension of the images
        dir_path is the path to the directory
        rescale is how much the pixel value will be divided, normally 1.0/255.0
    """
    all = []
    # Extract all name
    if type(ext) != list:
        ext = [ext]
    for e in ext:
        p = map(lambda x: str(x), pathlib.Path(dir_path).glob(e))
        # Interate through each name and load
        for path in p:
            # Load
            img = img_to_array(load_img(path))
            # Rescale pixel values
            img *= rescale
            if to_gray:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            all.append(img)
    return np.array(all)

def plot_side_by_side(im1, im2, im1_label='', im2_label=''):
    f, axarr = plt.subplots(1,2)
    axarr[0].imshow(im1)
    axarr[0].set_title(im1_label)
    axarr[1].imshow(im2)
    axarr[1].set_title(im2_label)
    plt.show()
