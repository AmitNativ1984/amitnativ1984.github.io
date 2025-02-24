---
layout: posts
title:  "Segment Anything Model Explained"
tagline: "SAM model overview"
date:   2025-02-24
categories: COMPUTER-VISION
permalink: /:categories/:title
header:
  overlay_image: 
  overlay_filter: 0.3
toc: true
created: true
classes: wide
---

Segment Anything model, is a foundation towards solving pormptable visual segmentation in images, developed by META. 

![segment-anything](/assets/images/2025-02-24-Segment-Anything-Model/SAM-results.jpeg)

In a nutshell, the model consists of:
1. **Image Encoder** (masked autoencoder) to extract image embeddings.
2. **Prompt Encoder** that takes in different types of prompts and encodes them into a fixed-length vector.
3. **Mask Decoder** to build a mask.

![SAM-model](/assets/images/2025-02-24-Segment-Anything-Model/SAM-model.png)

# **Image Encoder**
An image autoencoder is a model that takes an image as input and outputs a reconstructed image. The model is trained to minimize the difference between the input and the output image, usually with $L_2$ loss. The encoder's output is a low-dimensional representation of the input image, which is called an embedding. The decoder takes this embedding and reconstructs the image. 

![autoencoder](/assets/images/2025-02-24-Segment-Anything-Model/autoencoder.png)

The core of the model is a **Masked Autoencoder** which utilizes a vision transformer to achieve high scalability. Masked Autoencoders are execlent at learning image embedding at a patch level.

## **Masked Autoencoder**
Masked autoencoders were first introduced in the paper [Masked Autoencoders are Scalable Vision Learners](https://arxiv.org/abs/211.06377). Following the success of masked autoencoders for language processing. The idea is simple: Following ViT, and image is divided into regular non-opverlapping patches. Mask out a random subset of the input patches, and learn to reconstruct the full image.

MAE has two main components:

![MAE](/assets/images/2025-02-24-Segment-Anything-Model/MAE.png)

### 1. **MAE Encoder**

Using ViT, the image is projected into patches. Before entering the encoder, a large random subset of the patches (e.g. 75%) is masked out. Thus, the encoder is applied only to the remaining (e.g. 25%) *visible patches*. 

### 2. **MAE Decoder**
The input to the MAE decoder is the full set of tokens consisting of both token of the visible patches and tokens of the masked patches. *Each mask token is a shared, learned vector that indicates the presence of a missing patch to be predicted*. Positional embeddings are added to *all* tokens. Without the positional embeddings, the model would have no information about the location of the masked tokens in the image.
The decoder is only used during pre-training to preform the image construction task. 


In SAM, only the encoder part of MAE is used. The transformer used in ViT-H/16 - a very large transformer with 16x16 patch size. 

For a typical input image of size 1024x1024x3, the encoder transofmers the image into a 64x64x256 embedding (downscaling the image by x16, to 64x64 patches with 256 channels each).

![MAE Examples](/assets/images/2025-02-24-Segment-Anything-Model/MAE-examples.png)

# **Prompt Encoder**
There are two types of prompts use in SAM:

![prompt-encoder](/assets/images/2025-02-24-Segment-Anything-Model/prompt-encoder.png)

1. **Sparse**: *points*, *boxes* and *text*.

#### Points: 
Represented as a sum of positional encoding of the points location, and a learned embedding to incdicate either a **foreground point** or a **background point**.

#### Boxes:
Represented by an **embedding pair**. The positoinal encoding of its top-left corener is usmmed with a learned embedding represitng `top-left-corner`, and similarly for the `bottom-right-corner`.

#### Text:
Represented by **CLIP**. The authors use CLIP without any modifications, and therefore any text encoder can be used.

2. **Dense**: *segmentation masks*.
Masks hava spatial correspondesnce with the images. A small network is used to encode the mask into an embedding represetation of dimension 256. The mask embedding is then added element-wise to the image embedding. In case where no mask prompt is provided, a learned embedding, representing `no-mask` is added to each image emebdding location.

These embeddings are then added to the MAE embeddings.

# Light Weight Mask Decoder
The mask decoder is responsible to take the prompot encoder embeddings and tokens, and the image encoder embdeddings and tokes, and output the mask.

![MAE-model-blocks](/assets/images/2025-02-24-Segment-Anything-Model/MAE-model-blocks.png)

The mask decoder is a small network that takes the image embeddings (64x64x256) and the prompt embeddings ($N_{prompt}$ x256), and a **learned output token embedding**. This output embedding plays a pivotal role in the decoder's function, containing essential information required for the overall image segmentation task. As in image classifation, where the *cls token* encapsulates information about the overall image, in segemenation tasks, the output token servers as a critical element guiding the decoding process toweards the desired segmentation.

![Light-Weight-Mask-Decoder](/assets/images/2025-02-24-Segment-Anything-Model/light-weight-mask-decoder.png)

Each decoder layers performs four steps:
1. self-attention on the tokens.
2. cross-attention between token and image.
3. mlp updates each resulting token.
4. cross-attenstion between image embeddings (as queries) to tokens.
