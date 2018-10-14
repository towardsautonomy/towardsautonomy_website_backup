---
permalink: /deep_learning
---

# Deep Learning <img style="float: right;" src="/img/logo_circle.png" height="100" width="100">

###### Author: *[Shubham Shrivastava](http://www.towardsautonomy.com/#shubham)*   

---

<p align="center">
  <img src="/docs/dl/img/semseg_cover.png">
</p>

Deep learning is a subset of AI and machine learning that uses multi-layered artificial neural networks to deliver state-of-the-art accuracy in tasks such as object detection, speech recognition, language translation and others [[1]](#references).

Deep learning differs from traditional machine learning techniques in that they can automatically learn representations from data such as images, video or text, without introducing hand-coded rules or human domain knowledge. Their highly flexible architectures can learn directly from raw data and can increase their predictive accuracy when provided with more data.

In deep learning, each level learns to transform its input data into a slightly more abstract and composite representation. In an image recognition application, the raw input may be a matrix of pixels; the first representational layer may abstract the pixels and encode edges; the second layer may compose and encode arrangements of edges; the third layer may encode a nose and eyes; and the fourth layer may recognize that the image contains a face. Importantly, a deep learning process can learn which features to optimally place in which level on its own.

Deep Learning plays a huge role in Autonomous Driving. It is closely linked with computer vision and is used for applications like Road-Scene Segmentation, Object Detection, Driver Monitoring, or even End-to-End Self-Driving. Some of the terminologies very frequently used in deep learning are:

[DL = NN (Deep Learning = Neural Nets)](https://en.wikipedia.org/wiki/Deep_learning)   
DL is a subset of ML [(Machine Learning)](https://en.wikipedia.org/wiki/Machine_learning)    
[MLP: Multi layer Perceptron](https://en.wikipedia.org/wiki/Multilayer_perceptron)   
[DNN: Deep Neural Networks](https://deeplearning4j.org/neuralnet-overview)   
[RNN: Recurrent Neural Networks](https://en.wikipedia.org/wiki/Recurrent_neural_network)   
[LSTM: Long Short Term Memory](https://en.wikipedia.org/wiki/Long_short-term_memory)   
[CNN: Convolutinal Neural Networks](https://en.wikipedia.org/wiki/Convolutional_neural_network)   
[DBN: Deep Belief Networks](https://en.wikipedia.org/wiki/Deep_belief_network)   

A good read: [Deep Learning, By Ian Goodfellow, Yoshua Bengio and Aaron Courville](https://mitpress.mit.edu/books/deep-learning)

Some of the applications of deep learning in self-driving are presented here in details along with code for reference. Please contact [Shubham Shrivastava](http://www.towardsautonomy.com/#shubham) for any questions or concerns.

1. [Object Detection, Classification, and Localization](/dl/obj_detection)
1. [SSNet for Semantic Segmentation](/dl/semseg)
1. [Traffic Sign detection using CNN](/cv/traffic_sign_detection)
1. [Behavioral Cloning](/docs/dl/behavioral_cloning)

#### References

[1] https://developer.nvidia.com/deep-learning
[2] https://arxiv.org/pdf/1511.00561.pdf
