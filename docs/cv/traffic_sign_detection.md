---
permalink: /cv/traffic_sign_detection
---

## Traffic Sign Classification using CNN <img style="float: right;" src="/img/logo_circle.png" height="100" width="100">

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*   

---

Overview
---
Traffic signs are an integral part of our road infrastructure. They provide critical information, sometimes compelling recommendations, for road users, which in turn requires them to adjust their driving behaviour to make sure they adhere with whatever road regulation currently enforced. For Autonomous Driving, the vehicle must know about the surrounding infrastructure like maximum allowed speed, stop signs, yield signs. These require them to be able to perceive traffic signs and understand the surroundings. Traffic Sign Classification plays a major role in deciding the behavior of self-driving cars and helps them prepare for events like pedestrian crossings in advance.

Here, deep neural networks and convolutional neural networks are used to classify traffic signs. I used train and validate a model so it can classify traffic sign images using the [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset). After the model is trained, I tried the model on images of German traffic signs that I found on the web.

**Detailed code with explanation is given [here](/docs/cv/Traffic_Sign_Classifier.html)**

## The complete process of traffic sign detection can be broken down into:
* Load the data set
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report

## Dataset Exploration
* I loaded the Training, Validation, and Test data from a "Python pickled" file which was prepared from [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset). These pickled files can be downloaded here: [Training dataset - train.zip](/docs/cv/train.zip), [Validation dataset - valid.p](/docs/cv/valid.p), and [Test dataset - test.p](/docs/cv/test.p)
* The pickled data is a dictionary with 4 key/value pairs:
  - 'features' is a 4D array containing raw pixel data of the traffic sign images, (num examples, width, height, channels).
  - 'labels' is a 1D array containing the label/class id of the traffic sign. The file [signnames.csv](/docs/cv/signnames.csv) contains id -> name mappings for each id.
  - 'sizes' is a list containing tuples, (width, height) representing the original width and height the image.
  - 'coords' is a list containing tuples, (x1, y1, x2, y2) representing coordinates of a bounding box around the sign in the image. **THESE COORDINATES ASSUME THE ORIGINAL IMAGE. THE PICKLED DATA CONTAINS RESIZED VERSIONS (32 by 32) OF THESE IMAGES**
* The dataset contains 43 different classes of German Road Traffic Signs. Their distributions are given below.

  ![Training Dataset](/docs/cv/img/traffic_sign_detection/training_dataset.PNG)

  ![Validation Dataset](/docs/cv/img/traffic_sign_detection/validation_dataset.PNG)

  ![Testing Dataset](/docs/cv/img/traffic_sign_detection/testing_dataset.PNG)

* The images in dataset are 32x32.
* To visualize the training dataset, I picked up 5 random images from each classes and displayed it as shown below

  ![Training Dataset](/docs/cv/img/traffic_sign_detection/sample_training_dataset.PNG)

* I worked with RGB, Grayscale, and YUV colorspace for training and validating my ConvNet architecture however the YUV colorspace seemed to work the best. So, I converted all the datasets to YUV before feeding it through my Convolutional Neural Network.

## Design and Test of the Model Architecture

#### Convolutional Neural Network Architecture

  ![Convolutional Neural Network Architecture](/docs/cv/img/traffic_sign_detection/ConvNetArchitecture.png)

* The figure above shows the Convolutional Neural Network architecture I used for training, validation and testing the dataset.
* One thing to note here is that I provide the output of Layer 2 to the Fully Connected Layer as well. This was inspired by the paper [*"Traffic Sign Recognition with Multi-Scale Convolutional Networks", by Pierre Sermanet and Yann LeCun*](http://yann.lecun.com/exdb/publis/pdf/sermanet-ijcnn-11.pdf) and it definitely improved the performance of my ConvNet by about 1.5~2%
* I used *early termination* technique by setting my validation accuracy threshold to 94%. Although the maximum EPOCHS I set was 30, my network training was completed in 14 EPOCHS because of early termination.
* Running the training and validation set through my network with a batch size of 128 at a learning rate of 0.00075 yielded the accuracy of 99.491% on training dataset, and 94.399% on the validation dataset.

  ![Training and Validation Accuracy](/docs/cv/img/traffic_sign_detection/training_validation_accuracy.PNG)

* I used RELU activation function for intermediate layers and softmax at the output layer to compute logits. Adam Optimizer was further used to minimize the loss.
* To avoid overfitting, I used dropout regularization at Layer 4 and Layer 5 with a *"keep probability"* of 0.4.
* Once I had enough confidence in my network, I ran the test pickled dataset provided as a part of the project which resulted in 93.310% accuracy.
* I then downloaded 5 German Traffic Sign images from *"Google Images"* and fed them through the network. I obtained a 100% accuracy on these datasets.

#### Model Certainty
* I obtained the top 5 Softmax probabilities for each of 5 test images downloaded from web. The model seems to classify the dataset with a very high certainty (close to 100%)

```
Top 5 Softmax Probabilities For Each Image

TopKV2(values=array([[9.9999571e-01, 3.4213137e-06, 6.6664717e-07, 1.6374592e-07,
        4.1009054e-09],
       [1.0000000e+00, 2.3870503e-18, 3.9513720e-20, 8.8137774e-21,
        2.0161452e-22],
       [9.9990809e-01, 9.1885435e-05, 4.5486659e-17, 9.5398699e-18,
        4.3713207e-18],
       [1.0000000e+00, 1.0980637e-29, 0.0000000e+00, 0.0000000e+00,
        0.0000000e+00],
       [1.0000000e+00, 1.4886212e-22, 4.2423085e-28, 4.1486727e-34,
        5.7359064e-38]], dtype=float32), indices=array([[25, 29, 26, 20, 24],
       [ 1,  0,  2, 32,  6],
       [36, 38, 12, 41, 32],
       [38, 40,  0,  1,  2],
       [39, 31, 33, 21,  4]], dtype=int32))
```

![](/docs/cv/img/traffic_sign_detection/test_detection.png)

#### Feature Maps visualization

* We can also visualize the Neural Network's Feature Maps to understand what the weights at each layer look like

  ![Layer 1 Feature Maps](/docs/cv/img/traffic_sign_detection/layer1_feature_maps.PNG)

  ![Layer 2 Feature Maps](/docs/cv/img/traffic_sign_detection/layer2_feature_maps.PNG)

  #### Suggested Improvements

  **Optimizers**  
   To know more about optimizers, check [this article](http://sebastianruder.com/optimizing-gradient-descent/index.html#adam) for a nice description and comparison of different algorithms.

  **ConvNet**  
  To learn more about Convolutional Neural Networks, read [this book](http://www.deeplearningbook.org/contents/convnets.html)

  **To Increase Robustness**  
  For increased robustness, you can use the image augmentation technique to further rebalance the number of examples for each class and to expose your model to a wider variety of image qualities.

  Check out [this article](https://medium.com/@vivek.yadav/dealing-with-unbalanced-data-generating-additional-data-by-jittering-the-original-image-7497fe2119c3#.wvp4g6hle) for a great explanation with examples.
