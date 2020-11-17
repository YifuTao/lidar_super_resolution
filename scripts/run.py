#!/usr/bin/env python
from model import *
from data import *
import time
import sys

def train():
    
    # print('Load training data...  ')
    training_data_input, training_data_pred_ground_truth = load_train_data()

    # print('Compiling model...     ')
    model, model_checkpoint, tensorboard = get_model('training')

    # print('Training model...      ')
    model.fit(
              training_data_input,
              training_data_pred_ground_truth,
              batch_size=64,
              validation_split=0.1,
              epochs=100,
              verbose=1,
              shuffle=True,
              callbacks=[model_checkpoint, tensorboard]
             )

    model.save(weight_name)


def MC_drop(iterate_count=12):  #?

    test_data_input, _ = load_test_data()   # (8825,16,1024,1)
    # load model
    model, _, _ = get_model('testing')
    model.load_weights(weight_name)

    this_test = np.empty([iterate_count, image_rows_low, image_cols, channel_num], dtype=np.float32)
    test_data_prediction = np.empty([test_data_input.shape[0], image_rows_high, image_cols, 2], dtype=np.float32)

    for i in range(test_data_prediction.shape[0]):

        # print('Processing {} th of {} images ... '.format(i, test_data_prediction.shape[0]))
        sys.stdout.write("\rProcessing %dth /%d" % (i, test_data_prediction.shape[0]))
        sys.stdout.flush()
        for j in range(iterate_count):
            this_test[j] = test_data_input[i]   # 12,16,1024,1

        this_prediction = model.predict(this_test, verbose=1)   # 12,64,1024,1

        this_prediction_mean = np.mean(this_prediction, axis=0) # 64,1024,1
        this_prediction_var = np.std(this_prediction, axis=0)   # 64,1024,1
        test_data_prediction[i,:,:,0:1] = this_prediction_mean  # 8825,64,1024,2
        test_data_prediction[i,:,:,1:2] = this_prediction_var


    np.save(save_path,test_data_prediction)
    # np.save(os.path.join(home_dir, 'Documents', project_name, 'my' + test_set + '-' + model_name + '-from-' + str(image_rows_low) + '-to-' + str(image_rows_high) + '_prediction.npy'), test_data_prediction)


if __name__ == '__main__':
 
    # -> train network
    # train()

    # -> Monte-Carlo Dropout Test
    MC_drop()
    
