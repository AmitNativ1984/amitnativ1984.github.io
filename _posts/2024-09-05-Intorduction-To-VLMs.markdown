---
layout: posts
title:  "Intorduction to Vision Language Models"
tagline: "CLIP and Relatives, Losses and Transfer Learning"
date:   2024-09-05
categories: COMPUTER-VISION
permalink: /:categories/:title
header:
  overlay_image: /assets/images/VLMs/clip_arch.png
  overlay_filter: 0.3
toc: true
created: true
classes: wide
---

Vision langauge models (VLMs) are models that can understand and process both vision and text modalities. The joint understanding of both modalities lead VLMs to perform various tasks efficiently, like Visual Question Answering, Text-to-Image search etc.
At their core, VLMs find ways to map text and image pairs to a joint embedding space where each text-image pair is present as an embedding, so that the embedding of the image and text with similar meaning lie close together. 

![shared embedding space](https://blog.dataiku.com/hs-fs/hubfs/Screenshot%202022-12-30%20at%2013.43.35.png?width=600&height=301&name=Screenshot%202022-12-30%20at%2013.43.35.png)

