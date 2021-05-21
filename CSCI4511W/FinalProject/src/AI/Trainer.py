import keras
from keras import layers

import matplotlib.pyplot as plt

import tensorflow as tf
import sys
from datetime import datetime

import DataProcessor as DP


class Regress32x128Color00PastTrainer:
    def __init__(self):
        pass

    def train(self, arch, data_dir, epochs, batch_size, patience, plot_log):
        # Network
        network = arch.network
        # Loader
        loader = arch.data_loader
        # Load data
        X, Y = loader.load(data_dir)
        # Announce total sample
        print('====================================================\n====================================================\n')
        print(f'Training with total number sample = {Y.shape[0]}')
        print('====================================================\n====================================================\n')
        # Split data to train valid and test
        x_train, y_train, x_valid, y_valid, x_test, y_test = DP.dataset_split(X, Y)
        print('====================================================\n====================================================\n')
        print(f'Split to Train = {x_train.shape[0]}, Validation = {x_valid.shape[0]}, Test = {x_test.shape[0]} ')
        print('====================================================\n====================================================\n')
        # Train
        early_stop_callback = tf.keras.callbacks.EarlyStopping(monitor='loss', patience=patience) # Callback to monitor the loss and stop when the loss stop decreasing
        history = network.fit(x_train, y_train,
            epochs=epochs,
            batch_size=batch_size,
            shuffle=True,
            validation_data=(x_valid,y_valid),
            callbacks=[early_stop_callback ,tf.keras.callbacks.TensorBoard(log_dir=f'./log/{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}', histogram_freq=0, write_graph=False)]
        )

        # Test the network
        network.evaluate(x_test, y_test,
            batch_size=1
        )

        # Save the model
        network.save(f'./network_save/{type(arch).__name__}_{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}')

        # Plot log
        if plot_log:
            plt.figure(figsize=(8, 8))
            plt.title("Learning curve")
            plt.plot(history.history["loss"], label="loss")
            plt.plot(history.history["val_loss"], label="val_loss")
            # plt.plot( np.argmin(history.history["val_loss"]), np.min(history.history["val_loss"]), marker="x", color="r", label="best model")
            plt.xlabel("Epochs")
            plt.ylabel("log_loss")
            plt.legend()
            # plt.savefig('loss_ver1_GW_and_noise.png')
            # plt.close()
            plt.show()

# def load_data():
#     """
#         This function takes 
#         -train : path to train data
#         -validation : path to validation data
#         -test : path to test data
#         -g: gray means converting train image to gray, otherwise, keep original
#     """
#     args = sys.argv[:]
#     args.pop(0)
#     # Default values of input arguments
#     train_folder = ''
#     validation_folder = ''
#     test_folder = ''
#     im_w = 128
#     im_h = 128
#     rbg_to_gray = False
#     # Get input arguments
#     if len(args) < 2:
#         print('Invalid number of input arguments')
#         sys.exit(-1)
#     else:
#         # Getting arguments from terminal
#         while len(args) > 0:
#             # Get value
#             vl = args.pop().strip()     # value
#             cd = args.pop().strip()     # command
#             if cd == '-train':
#                 # input value
#                 train_folder = vl
#             elif cd == '-validation':
#                 validation_folder = vl
#             elif cd == '-test':
#                 test_folder = vl
#             elif cd == '-w':
#                 # width
#                 im_w = int(vl)
#             elif cd == '-h':
#                 im_h = int(vl)
#             elif cd == '-g':
#                 rbg_to_gray = True if vl == 'gray' else False
#         if train_folder == '' or validation_folder == '' or test_folder == '':
#             print('Invalid arguments')
#             sys.exit(-1)
#         # Load all images in train folder
#         x_train = load_all_image_from_folder(train_folder, to_gray=rbg_to_gray)
#         x_validation = load_all_image_from_folder(validation_folder, to_gray=rbg_to_gray)
#         x_test = load_all_image_from_folder(test_folder, to_gray=rbg_to_gray)
#         return x_train, x_validation, x_test

# #============== main ============
# print("Num GPUs Available: ", len(tf.config.experimental.list_physical_devices('GPU')))

# # Get image to train
# x_train, x_validation, x_test = load_data()
# # Build the autoencoder full net
# network = AutoencoderVer3().network

# # Train the network
# early_stop_callback = tf.keras.callbacks.EarlyStopping(monitor='loss', patience=3) # Callback to monitor the loss and stop when the loss stop decreasing
# network.fit(x_train, x_train,
#     epochs=20,
#     batch_size=100,
#     shuffle=True,
#     validation_data=(x_validation,x_validation),
#     callbacks=[early_stop_callback ,tf.keras.callbacks.TensorBoard(log_dir=f'./log/{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}', histogram_freq=0, write_graph=False)]
# )

# # Test the network
# network.evaluate(x_test, x_test,
#     batch_size=1
# )

# # Save the model
# network.save(f'./save/{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}')