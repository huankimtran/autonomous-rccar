import keras
from keras import layers
import tensorflow as tf
from tensorflow.keras.models import Model
import os
import pathlib

import DataProcessor as DP
import Trainer as TR

"""
    Network Architecture
    
"""
class Regress32x128Color00PastVer0:
    """
        Input color img size 32x128 heightxwidth
        No infor about previous action
    """
    def __init__(self, weight_folder=None):
        self.network = self.build() if weight_folder == None else keras.models.load_model(weight_folder)
        self.data_loader = DP.Regress32x128Color00PastDataLoader()
        self.trainer = TR.Regress32x128Color00PastTrainer()

    def build(self):     
        # Network architecture to train the dense auto encoder
        input_img = keras.Input(shape=(32, 128, 3)) # 32x128x3 (w*h*rbg)

        x = layers.Conv2D(32, (3, 3), activation='relu', padding='same')(input_img) # 32x128x32
        x = layers.MaxPooling2D((2, 2), padding='same')(x)  # 16x64x32
        x = layers.Conv2D(64, (3, 3), activation='relu', padding='same')(x) # 16x64x64
        x = layers.MaxPooling2D((16, 16), padding='same')(x)         # 1x4x64
        x = layers.Flatten()(x)

        x = layers.Dense(120, activation='relu') (x)
        x = layers.Dropout(.5)(x)
        x = layers.Dense(60, activation='relu') (x)
        x = layers.Dropout(.2)(x)
        x = layers.Dense(20, activation='relu') (x)

        output = layers.Dense(2, activation='linear') (x)

        network = keras.Model(input_img, output)
        network.compile(optimizer='adam', loss='mean_squared_error')
        return network
    
    def train(self, train_data_dir, epochs=20, batch_size=60, patience=5, plot_log=False):
        self.trainer.train(self, train_data_dir, epochs=epochs, batch_size=batch_size, patience=patience, plot_log=plot_log)

    def predict(self, X):
        # Make sure the input data is normalized
        if not self.data_loader.input_is_normalized(X):
            X = self.data_loader.input_normalize(X.copy())
        # Get the prediction from the network
        network_output = self.network.predict(X) 
        # The trained labels are are normalized so let unormalize the output from the network 
        mapped_output = self.data_loader.output_unnormalize(network_output)
        return mapped_output



class Regress32x128Color00PastVer1:
    """
        NO DROPOUT
        Input color img size 32x128 heightxwidth
        No infor about previous action
    """
    def __init__(self, weight_folder=None):
        self.network = self.build() if weight_folder == None else keras.models.load_model(weight_folder)
        self.data_loader = DP.Regress32x128Color00PastDataLoader()
        self.trainer = TR.Regress32x128Color00PastTrainer()

    def build(self):     
        # Network architecture to train the dense auto encoder
        input_img = keras.Input(shape=(32, 128, 3)) # 32x128x3 (w*h*rbg)

        x = layers.Conv2D(32, (3, 3), activation='relu', padding='same')(input_img) # 32x128x32
        x = layers.MaxPooling2D((2, 2), padding='same')(x)  # 16x64x32
        x = layers.Conv2D(64, (3, 3), activation='relu', padding='same')(x) # 16x64x64
        x = layers.MaxPooling2D((16, 16), padding='same')(x)         # 1x4x64
        x = layers.Flatten()(x)

        x = layers.Dense(120, activation='relu') (x)
        x = layers.Dense(60, activation='relu') (x)
        x = layers.Dense(20, activation='relu') (x)

        output = layers.Dense(2, activation='linear') (x)

        network = keras.Model(input_img, output)
        network.compile(optimizer='adam', loss='mean_squared_error')
        return network
    
    def train(self, train_data_dir, epochs=20, batch_size=60, patience=5, plot_log=False):
        self.trainer.train(self, train_data_dir, epochs=epochs, batch_size=batch_size, patience=patience, plot_log=plot_log)

    def predict(self, X):
        # Make sure the input data is normalized
        if not self.data_loader.input_is_normalized(X):
            X = self.data_loader.input_normalize(X.copy())
        # Get the prediction from the network
        network_output = self.network.predict(X) 
        # The trained labels are are normalized so let unormalize the output from the network 
        mapped_output = self.data_loader.output_unnormalize(network_output)
        return mapped_output


# Scratch paper down here
if __name__ == '__main__':
    data_path = str(pathlib.Path(os.path.realpath(__file__)).parent.parent.parent.joinpath('data/map_2_merged_left_right/'))
    arch = Regress32x128Color00PastVer0()
    arch.train(data_path, plot_log=True)
