# Dimmer: Self-Adaptive Network-Wide Flooding with Reinforcement Learning

To cite the paper, please use (Publication to appear, will be updated once the proceedings are out):
> V. Poirot, O. Landsiedel,  “Dimmer: Self-Adaptive Network-Wide Flooding with Reinforcement Learning,”   in the IEEE International  Conference on Distributed Computing Systems (ICDCS), 2021. 
[Direct link TBA](https://www.github.com/ds-kiel/dimmer)

Dimmer is a self-adaptive flooding protocol for low-power wireless networks. Dimmer uses an embedded Deep Q-Network (DQN) and multi-Armed Bandits (MAB) to decide how to adapt its network configuration at runtime to (1) tackle and adapt to external interference, and (2) deactivate devices to save energy under optimal conditions.
Dimmer builds on the [Low-power Wireless Bus (LWB)](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/LWBSenSys12.pdf).


## Abstract
The last decade saw an emergence of Synchronous Transmissions (ST) as an effective communication paradigm in low-power wireless networks. Numerous ST protocols provide high reliability and energy efficiency in normal wireless conditions, for a large variety of traffic requirements. Recently, with the EWSN dependability competitions, the community pushed ST to harsher and highly-interfered environments, improving upon classical ST protocols through the use of custom rules, hand-tailored parameters, and additional retransmissions. The results are sophisticated protocols, that require prior expert knowledge and extensive testing, often tuned for a specific deployment and envisioned scenario. In this paper, we explore how ST protocols can benefit from self-adaptivity; a self-adaptive ST protocol selects itself its best parameters to (1) tackle external environment dynamics and (2) adapt to its topology over time.
We introduce Dimmer as a self-adaptive ST protocol. Dimmer builds on LWB and uses Reinforcement Learning to tune its parameters and match the current properties of the wireless medium. By learning how to behave from an unlabeled dataset, Dimmer adapts to different interference types and patterns, and is able to tackle previously unseen interference. With Dimmer, we explore how to efficiently design AI-based systems for constrained devices, and outline the benefits and downfalls of AI-based low-power networking. We evaluate our protocol on two deployments of
resource-constrained nodes achieving 95.8\% reliability against strong, unknown WiFi interference. Our results outperform baselines such as non-adaptive ST protocols (~27%) and PID controllers, and show a performance close to hand-crafted and more sophisticated solutions, such as Crystal (~99%). 

## How to use

##### Setup


To compile and flash Dimmer's image on embedded platforms, please follow the [Contiki-NG setup tutorial](https://github.com/contiki-ng/contiki-ng/wiki).

To compile the Sky image, please go to the `dimmer/apps/dimmer-app` folder and use the following commands: `make clean TARGET=sky && make dimmer-app TARGET=sky -j4`

To convert the image in hex format, please use `msp430-objcopy dimmer-app.sky -O ihex dimmer-app.ihex`

To run Dimmer on DCube, use the `dimmer/apps/dimmer-dcube` application.

##### Embedded Implementation 

The embedded implementation of dimmer is located in the `dimmer/` folder.
The actual network protocol is located in `dimmer/os/net/mac/dimmer`, while the main application is in `dimmer/apps/dimmer-app`.
Currently,  Dimmer only supports the Tmote Sky mote.

##### Training and Testing Environement

The training environment is located in the `gyms/` folder, and is an instance of an [OpenAI Gym](https://gym.openai.com/) environment. 

To install the training environment, go to `gyms/` and run `pip install -e .`. This will install the 'dimmer' python package.

The training and evaluation scripts are located in `RL/`, and are written in Python.
The training scripts require the presence of [OpenAI's baselines](https://github.com/openai/baselines).



## Disclaimer 
> Although we tested the code extensively, Dimmer is a research prototype that likely contain bugs. We take no responsibility for and give no warranties in respect of using the code.

Unless explicitly stated otherwise, all Dimmer sources are distributed under the terms of the [3-clause BSD license](license). This license gives everyone the right to use and distribute the code, either in binary or source code format, as long as the copyright license is retained in the source code.

This repository contains an implementation of Dimmer using the [Contiki-NG](http://contiki-ng.org/) operating system and builds on the implementation of the [Low-power Wireless Bus](https://github.com/ETHZ-TEC/LWB-Baseline) protocol.
