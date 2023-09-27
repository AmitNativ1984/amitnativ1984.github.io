---
layout: posts
title:  "BLOOD PRESSURE ESTIMATION FROM PPG SIGNALS USING CONVOLUTIONAL NEURAL NETWORKS AND SIAMESE NETWORK"
date:   2022-12-16 17:33:00 +0200
categories: INTERESTING
usemathjax: true
author_profile: true
# permalink: /posts/
published: false
---

An overview of the paper: 
[Blood Pressure Estimation from PPG Signals Using Convolutional Neural Networks and Siamese Network](https://ieeexplore.ieee.org/document/9053446)

## Main takeaways:
1. Photoplethysmography (PPG) is a non-invasive method that can measure blood volume chagnes in the microvascular bed of the skin, by illuminating the skin with a light source and measuring changes in light absorption.
2. PPG signals are prone to significant artifacts that affect measuremt accuracy.
3. The paper proposes two methods relaying on CNNs to estimate blood pressure from PPG signals, as well as methods to preprocess the data to improve results. The two methods are:
   1. Simple CNN which is calibration free.
   2. Siamese CNN which achieves more accurate measurements by estimating BP changes with respect to patient's PPG and **ground truth PPG** at calibration time. 
4. The siamese network method achieves results as good as many home BP devidces.


## INTROCDUCTION:
---
There are two types of blood pressure: 
   - Systolic: when the heart beats and BP is at its highest.
   - Diastolic: Between heart beats, when BP is at its lowest.
  
The normal BP for an adult is 120/80 mmHg (systolic/diatolic).
There are two types of BP measurements:
  1. Invasive: A procedure called invasive aretial line. A catheter is inserted into an artery and the blood pressure is measured directly.
  1. Non-invasive: A cuff is placed around the arm and the pressure is measured by a pressure sensor. The cuff is inflated and deflated to measure the systolic and diastolic BP. *These methods are not suitable for long-tern ambulatory BP monitoring due to discomfort*

Photoplethysmogaphy (**PPG**) is an optically obtained signal that can be used to measure bolled volume changes in the microvascular bed of tissue. It is obtained by illuminating the skin and measuring changes in light abbsourption. It can be obtained by a pulseoximeter, or by a smartwatch.

**Why PPG**:
- Measures: BP, heart rate, oxygen.
- Cheap, non-invasive, and easy to use.

**Problems with PPG**:
- Prone to signifcant artifacts that affect measurment naccuracy.


## **DATASET PREPROCESSING**:
---
The dataset is taken from [MIMIC-II waveform database]. The authors use two signals: **PPG** and correspondent **arterial blood pressure (ABP)**, recorded from patients in intensive care. The CNN are trained with the PPG signal as input and given the ABP as ground truth.
Accurate CNN predictions require reliable ground truth, so great attention is given to cleaning up the data.

The authors split the PPG and ABP signals into **30 - seconds windows**. However, these windows often contain artifacts that can damage the CNN prediction (patient falloff, respretory movement, loosing contact...). There are three types of artifacts that disqaulify a measurement window:
> 1. (a) Unreasonable systolic (too high) or diastolic (too low) measurements or even negative values.
> 2. (b) Unstable PPG or ABP meaurements over the 30-sec window.
> 3. (c) Noisy PPG or ABP signals - seen as local "peaks" in the waveform.

![BP PPG artifacts]

The data is preprocessed in the following manner, step by step:

> 1. **Meaure normalized auto-correlation**. If the normalized auto-correlation is low, the signal is unstable (noisy, BP/PPG unstable).
> 2. **Remove unreliable patients**: patients that don't have enough windows after the prev step.
> 3. **For every patient remove all outlier windows**  windows with values too far appart from patient first window.
> 4. **The patients are separated to train/val/test**, rather than windows.

Finally, each PPG 30-sec window is converted into a spectogram. The spectogram is then pushed to the CNN.
![Spectogram]



## CALIBRATION FREE BP ESTIMATION
---
The first CNN described in the paper is AlexNet with regression head.
The network is trained by feeding the spectogram to the NN and predicting the ground truth BP.

The original AlexNet:
![AlexNet-Regression]

- **The Loss function**: \\[L_1 = |BP - BP'|\\]
 Which is good for regression tasks (both predicting both positive and negative values)


## BP ESTIMATION WITH SIAMESE NETWORK:
---
The main idea is this architecture, is not to estiamte the BP directly, but rather estimate the difference from a calibrated PPG. 
The Calibrated PPG is taken as the first 30-sec window with a ground truth BP measure. 
**The output of the siamse NN is BP difference from a calibrated BP value**

### Siamese network architecture:
- The same AlexNet.
- Two identical CNN.
- Subtract their feature vector (prior to original regression head)
- Add non linearity and add regression head.
- The output is the BP **difference** with respect to the calibrated BP value.

**(The draw back is the need for a know true BP value during the calibratioin process)**

The difference from classical Siamese network:
* Regression network.
* Most siamese cnn use a metric with only positive values to estimate distances (like Euclidean distance. Here negtive values are allowed. 

![Siamese-NN]

## RESULTS AND THOUGHTS:
Below are the confusion matrices:


The confusion matrices of the Siamese NN have more values close to the diagonlae. So the second method is more accurate.
Also, the results of the Siamese NN are inside the MAD and STD defined values for home BP measurement accuracy requirements ($$MAD<5$$, $$STD<8$$).
Also, the diastolic BP estimation is more accurate with respect to systolic BP estimation, due to the lower variance of diastolic measurements gt.

One major draw back for the second method is the need for calibrating with ground truth BP. The Siamese NN measures only the difference. **How is this gt value measured at home?**

![Confusion-matrices]
![Results]

[MIMIC-II waveform database]: https://archive.physionet.org/physiobank/database/mimic2wdb/
[BP PPG artifacts]: /assets/images/2022-12-16-blood-pressure-with-cnn/BP-and-PPG-artifacts.png
[Spectogram]: /assets/images/2022-12-16-blood-pressure-with-cnn/spectogram.png
[AlexNet-Regression]: /assets/images/2022-12-16-blood-pressure-with-cnn/AlexNet-Regression.png
[Siamese-NN]: /assets/images/2022-12-16-blood-pressure-with-cnn/siamese-nn.png
[Confusion-matrices]: /assets/images/2022-12-16-blood-pressure-with-cnn/confusion-matrices.png
[Results]: /assets/images/2022-12-16-blood-pressure-with-cnn/results.png
