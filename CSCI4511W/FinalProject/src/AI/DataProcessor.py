import numpy as np
import cv2
import pathlib
import random

from tensorflow.keras.preprocessing.image import load_img, img_to_array

def dataset_split(X,Y, train_ratio=0.8):
    """
        test_ratio = validation_ratio = (1 - train_ratio)/2
        return X_train, Y_train, X_valid, Y_valid, X_test, Y_test
    """
    n_sample = Y.shape[0]
    # Create the index list and randomize it to create random training dataset
    index_list = list(range(n_sample))
    random.shuffle(index_list)
    # Get the train set
    test_index = int(len(index_list)*train_ratio)
    X_train = np.take(X, index_list[:test_index], axis=0)
    Y_train = np.take(Y, index_list[:test_index], axis=0)
    # Get the validation set
    validation_index = int(len(index_list) * (train_ratio + (1.0 - train_ratio)/2))
    X_valid = np.take(X, index_list[test_index:validation_index], axis=0)
    Y_valid = np.take(Y, index_list[test_index:validation_index], axis=0)
    # Get the test set
    X_test = np.take(X, index_list[validation_index:], axis=0)
    Y_test = np.take(Y, index_list[validation_index:], axis=0)
    return X_train, Y_train, X_valid, Y_valid, X_test, Y_test

class Regress32x128Color00PastDataLoader:
    def __init__(self):
        """
        No parameter to save yet
        """
        
    def load(self, dir_path, ext=['*.jpg'], to_gray=False):
        """
            ext is extension of the images
            dir_path is the path to the directory
            rescale is how much the pixel value will be divided, normally 1.0/255.0
            to_gray is wheter or not you want to load image as gray image

            images in folder has name formated as
            Index_SteerLevel_ThrottleLevel.jpg
            This function will go through each image in the folder
            - Load the image
            - Extract the labeled data from the image name
            - Return X,Y = load(path)
                - X: the list of input images
                - Y: the output [(steer, throttle), ..] 
            Becareful 1 entry in X associate with 1 entry in Y but these entry are not sorted by any order.
            Y value is rescale to -1 to 1 by dividing the decoded value by 100
        """
        X = list()
        Y = list()
        # Extract all name
        if type(ext) != list:
            ext = [ext]
        for e in ext:
            p = list()
            # Extract data from image file name
            for x in pathlib.Path(dir_path).glob(e):
                # Extract name without extension
                n = x.name[:len(x.name)-len(e[1:])]
                # Extract data
                data = [float(k) for k in n.split('_')]
                # Add name of the list to the end of data
                data.append(str(x))
                # Add to list
                p.append(tuple(data))
            # Interate through each name and load
            for data in p:
                path = data[-1]
                # Load image
                img = img_to_array(load_img(path))
                # Convert to gray if needed
                if to_gray:
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
                # Save data
                X.append(img)
                # Save label
                Y.append(np.array((data[1:-1])))
        # Normalizing the dataset before returning
        norm_X = self.input_normalize(np.array(X))
        norm_Y = self.output_normalize(np.array(Y))
        return norm_X, norm_Y

    def output_normalize(self, data):
        # Label range is from [-100,100] so let resacle and shift to all positive number
        # [-100,100] -> [-1,1]
        return data/100
    
    def output_unnormalize(self, data):
        # Reversing the normalized data or the output from the network
        # [-1,1] -> [-100, 100]
        return data*100

    def input_normalize(self, data):
        # Normalize input data: normalize the pixel from [0,255] -> [0,1])
        return data/255

    def input_unnormalize(self, data):
        # unnormalize input data
        return data*255

    def input_is_normalized(self, data):
        # It is normalized if all number is in the range [0,1]
        # Count and only return True when none violated
        return np.sum(np.where(np.logical_or(data > 1, data < 0),1,0)) == 0


# Scratch paper down here
if __name__ == '__main__':
    a = Regress32x128Color00PastDataLoader()
    X, Y = a.load('../data/map_2_merged_left_right/')