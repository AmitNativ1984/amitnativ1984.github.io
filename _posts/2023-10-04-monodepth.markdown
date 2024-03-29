---
layout: posts
title:  "MONODEPTH"
tagline: "Self Supervised Depth and Pose Estmation"
date:   2023-10-04
categories: COMPUTER-VISION
permalink: /:categories/:title
header:
  overlay_image: /assets/images/monodepth/monodepth-header.jpeg
  overlay_filter: 0.3
toc: true
created: true
classes: wide
---

Depth and pose estimation are crucial for an autonomous robot to navigate in the environment. Classical methods, such as SLAM are based on feature matching and bundle adjustment. Recently, deep learning based methods have been proposed to estimate depth and pose. These methods use **self supervision** for training. That is, they do not use any ground truth labels, but instead use the data itself to supervise the training. For example - in the case of depth estimation, the left and right images of a stereo pair are used to supervise the training by calculating the disparity between the two images, and then creating the right image from the left based on the estimated disparity. This is called **left-right consistency**. At the end of training, the network estimates the depth of the scene, based on a single camera - identical to the left image of the stereo pair. Another method, uses a single monocular image stream to estimate the depth. This is done by estimating the pose between consecutive frames, and then using the pose to estimate the depth. This is called **pose supervision**. Finally, methods such as **SfM-Learner** use both left-right consistency and pose supervision to train the network.
Estimating depth from a single camera is a cost affective solution for robots, as it does not require a stereo camera or an expensive LiDar. However, it is more challenging. In this post we will discuss two papers that estimate depth from a single camera. One uses a stereo camera for self supervision, and the other uses pose supervision. 


# Unsuperised Monocular Depth Estimation with Left-Right Consistency

In this part, we will discuss the **monodepth** method, which uses a single image stream to estimate the depth of the scene. The method is based on the paper [Unsupervised Monocular Depth Estimation with Left-Right Consistency](https://arxiv.org/pdf/1609.03677.pdf) by Godard et al. 

The authors of the paper treat automatic depth estimation as an image reconstruction problem during training. During training, depth data is provided, but instead, the fully convolutional network is trained to synthesize depth as an intermediate step to predict pixel level correspondences between stereo image pairs that have a known baseline.

> **The network performs end-to-end unsupervised learning monocular depth estimation with a training  loss that enforces left-right depth consistency inside the network.**

It is important to note, that only minimizing the photometric loss cab result in good quality reconstructions, but poor depth estimation. To overcome this, the authors use left-right consistency to improve the quality of the depth estimation. 

## Methods
### Depth Estimation as Image Reconstruction
Given a calibrated pair of binocular cameras. if we can learn a function that is able to reconstruct one image from the other, then we have learned something about the 3D shape of the scene that is being image. 

Given a stereo pair, the disparity is the displacement of a pixel between the left and right images. The disparity is inversely proportional to the depth of the scene. The disparity is calculated as follows:

![disparity](/assets/images/monodepth/disparity.png)

![disparity2](/assets/images/monodepth/disparity2.png)



$$ 
d = \frac{f \cdot b}{Z}
$$

where $d$ is the disparity, $f$ is the focal length, $b$ is the baseline between the two cameras, and $z$ is the depth of the scene.

So, if we have a given left image $I^l$, and a disparity map that wraps the left image to the right image $d^r$, we can estimate the right image as $\tilde{I^r}=I^l(d^r)$.

### Depth Estimation Network
Naively, we can take the left image as input, learn to predict the disparity map $d^r$, and then sample the left image to generate the right image $\tilde{I^r}$. However, the result will be aligned with the right camera rather than the left camera, which is undesirable (this is shown as **Naive** below).

Another option will be to take the left image as input, learn to predict the disparity map $d^l$, and then sample the right image to generate the left image $\tilde{I^l}$ (this is shown as **No LR** in the image below). While this works, the inferred disparity and therefore depth suffer from more artifacts. 

The solution the authors propose is train the network to predict both $d^l$ and $d^r$ from the left image alone (this is shown as **Ours** in the image below). Both right and left images are sampled during training time. This is called **left-right consistency**. Enforcing left-right consistency during training improves the quality of the depth estimation.

| ![sampling-methods](/assets/images/monodepth/sampling-methods.png) | 
|:--:|
| *Sampling strategies: With naive sampling the CNN produces a disparity map aligned with the target instead of the input. No LR corrects for this, but suffers from artifacts. Our approach uses the left image to produce disparities for both images, improving quality by enforcing mutual consistency.* |

![LR-consistency](/assets/images/monodepth/LR-consistency.png)

As a final note, the network learns the disparity predictions at 4 different scales. All are considered for loss calculations. 

### Training Loss
The loss function $C_s$ at each scale $s$ forms the total loss as the sum:

$$
C = \sum_{s=1}^4 C_s
$$

The loss at each scale if a combination of three terms:

$$ 
C_s = \alpha_{ap} C_{ap} + \alpha_{ds} C_{ds} + \alpha_{lr} C_{lr}
$$

where $C_{ap}$ is the appearance matching loss - encourages the reconstructed image to appear similar to the training image. $C_{ds}$ is the disparity smoothness loss. $C_{lr}$ encourages the left and right disparities to be consistent. Each of the main terms contains left-right consistency loss. The $\alpha$ terms are weights that control the contribution of each loss term.

$$
C_{ap} = C_{ap}^l + C_{ap}^r
\\ C_{ds} = C_{ds}^l + C_{ds}^r
\\ C_{lr} = C_{lr}^l + C_{lr}^r
$$

#### **Appearance Matching Loss $C_{ap}$**
During training the network learns to generate an image by sampling pixels from the opposite stereo image. The sampling uses bilinear sampling to sample the input image using the disparity map. The appearance matching is a combination of the L1 loss and the SSIM loss.

$$
C_{ap}^l = \frac{1}{N} \sum_{i,j} \alpha \frac{1-SSIM(I_{ij}^l, \tilde{I}_{ij}^l)}{2} + (1-\alpha) \| I_{ij}^l - \tilde{I}_{ij}^l\|
$$

Where $SSIM$ is the structural similarity index, $\alpha$ is a parameter that controls the contribution of the SSIM loss, and $N$ is the number of pixels in the image. The right image loss is calculated in the same way.

#### **Disparity Smoothness Loss $C_{ds}$**
This loss encourages disparities to be locally smooth with an L1 penalty on the disparity gradients $\partial d$. Because normally we expect depth discontinuities around image edges, we weight the disparity smoothness loss by the image gradient magnitude $|\partial I|$:

$$
C_{ds}^l = \frac{1}{N} \sum_{i,j} |\partial_x d_{ij}^l| e^{-|\partial_x I_{ij}^l|} + |\partial_y d_{ij}^l| e^{-|\partial_y I_{ij}^l|}
$$

#### **Left-Right Consistency Loss $C_{lr}$**
To increase the accuracy of the disparity maps, the network learns to predict the left and right image disparities, *while only being given the left image at training time*. This cost attempts to make the left-view disparity equal to the *projected* right-view disparity map.

$$
C_{lr}^l = \frac{1}{N} \sum_{i,j} |d_{ij}^l - d_{ij + d^l_{ij}}^r|
$$

|![loss-function](/assets/images/monodepth/loss-function.png)|
|:--:|
| *Loss module outputs left and right disparities $d^l, d^r$. The loss combines smoothness, reconstruction, and left-right disparity consistency terms. **The loss module is calculated for every scale*** |

|![results](/assets/images/monodepth/monodepth-results.png)|
|:--:|
| *Qualitative results on the KITTI dataset.*|


# Unsupervised Learning of Depth and Ego-Motion from Video

In this part, we will discuss the **SfM-Learner** method, which uses a single image stream to estimate the depth of the scene. The method is based on the paper [Unsupervised Learning of Depth and Ego-Motion from Video](https://arxiv.org/pdf/1704.07813.pdf) by Zhou et al.

The authors propose a method to estimate depth and pose from a single monocular image stream. In stereo depth estimation, the depth is estimated by finding correspondences between left and right images, calculating the disparity, and then using the disparity to calculate the depth with a known camera baseline.
In a similar way, the authors propose to calculate the pose between consecutive frames, find correspondences between frames, and then use the pose to calculate the depth from disparity.

|![pose-estimation-supervision](/assets/images/monodepth/pose-estimation-supervision.png)|
|:--:|
| * Overview of the supervision pipeline. The depth network takes the target image $I_t$ and outputs the estimated depth map $\hat{D}_t$. The pose network takes both the target view $I_t$ and two nearby source images from the video stream $I_{t-1}$ and $I_{t+1}$. The output of the pose network is the relative pose transformations $\hat{T}_{t-1 \rightarrow t}$ and $\hat{T}_{t+1 \rightarrow t}$. The depth map, and calculated poses are then used to wrap the source images $I_{t-1}, I_{t+1}$ to the target image $I_t$*|

### Model limitations:
For monocular depth estimation to work, the following assumptions must be met:
1. **The scene is static** without moving objects.
2. **No occlusions** between target view and source view.
3. **Lambertian surfaces** only, so that photo-consistency error is meaningful.

Violations of these assumptions will corrupt the gradients in training.

#### Explainability Mask
To improve the robustness of the learning model, an additional ***explainability prediction*** is trained simultaneously with the depth and pose networks. The explainability prediction $\hat{E}_s$ indicates the network belief in where direct view synthesis is valid.

|![explainablity-mask](/assets/images/monodepth/explainablity-mask.png)|
|:--:|
| *The explainability mask $\hat{E}_s$. Notice high values around moving objects and occlusions. Those will not be considered for visibility loss*|


|![pose-estimation-supervision](/assets/images/monodepth/pose-estimation-explainability-mask-nn.png)|
|:--:|
| *NN architecture used*|

### Loss Function

#### 1. Photometric Loss
Given a set of source images $I_s \in <I_1,...,I_N>$ and a target image $I_t$, the photometric loss can be calculated as:

$$
\mathcal{L}_{vs} = \sum_{s} \sum_{p} \hat{E}_s(p) |I_t(p) - \hat{I}_s(p)|
$$

To find the relation between the target image pixel value $p_t$ and the source image pixel value $p_s$, the following calculation holds:

$$
p_s \sim K \hat{T}_{s \rightarrow t} \hat{D}_t(p_t) K^{-1} p_t
$$

Where $D_t$ is the depth value of that pixel. As the projected pixel $p_s$ is continuous, we use bilinear sampling to calculate the pixel value. This is also referred to as **differentiable image warping process**

|![bilinear-sampling](/assets/images/monodepth/bilinear-sampling.png)|
|:--:|
| $\hat{I}(p_t) = I_s(p_s) = \sum_{i\in{t,b},j\in{l,r}} w^{ij}I_s(p_s^{ij})$|

#### 2. Smoothness Loss
Similar to what was done in the previous method, the smoothness loss is calculated with weights based on the rgb image gradient magnitude (suppress loss around edges)

$$
\mathcal{L}_{smooth} = \sum_{s} \sum_{p} |\partial_x D(p)| e^{-|\partial_x I_s(p)|} + |\partial_y D(p)| e^{-|\partial_y I_s(p)|}
$$

#### 3. Explainability Loss
Since there is no direct supervision for $\hat{E}_s$, training with $\mathcal{L}_{vs}$  would result in a trival solution for the network to predict $\hat{E}_s = 0$. To resolve this, the authors propose to use a **cross entropy** loss with a constant label 1 at each pixel location. This will to encourage the network to predict $\hat{E}_s$ non-zero values. 

So the final loss is:

$$
\mathcal{L}_{final} = \sum_{s}\mathcal{L}_{vs} + \lambda_{smooth} \mathcal{L}_{smooth} + \lambda_{exp} \sum_{s}\mathcal{L}_{exp}
$$

![pose-results](/assets/images/monodepth/pose-results.png)